package org.firstinspires.ftc.teamcode.UnusedOpModes;

import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.closePos;
import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.openPos;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.ShootSystem;


@Disabled
@TeleOp(name = "SwapTele")
public class SwapTele extends LinearOpMode {
    private ShootSystem shooter;
    private boolean isShooting;

    // Toggle Variables
    private boolean isManualMode = false;
    private boolean lastRightStick = false;

    private double manualFlywheelPower = 0.5;
    private double manualAnglePos = 0.25;


    private int shootMode = 1;

    // Drive Vars

    private DcMotorEx lb;
    private DcMotorEx rb;
    private DcMotorEx lf;
    private DcMotorEx rf;

    private final double driveSpeed = 1;
    private final double acceleration = 4;
    private final double turnAccel = 3;

    private final double p = 0.021, i = 0.00001, d = 0.00011;
    private double lastError;
    private double iSum;


    private double lStickPosX;
    private double lStickPosY;
    private double rStickPosX;
    private final double stickClampMin = 0.3;
    private final double stickClampMax = 1;

    @Override
    public void runOpMode(){
        InitMotors();
        SetDriveDirection("forward");
        SetBrakes();
        shooter = new ShootSystem(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive()){


            boolean currentBackPress = gamepad2.back;

            if (gamepad2.rightStickButtonWasPressed() && !lastRightStick) {
                isManualMode = !isManualMode;
            }
            lastRightStick = gamepad2.rightStickButtonWasPressed();

            if (!isShooting || gamepad1.left_bumper)
                Drive();

            if (gamepad2.y){
                shooter.feeder.setPosition(closePos);
            } else {
                shooter.feeder.setPosition(openPos);
            }

            if (isManualMode) {

                // manual stick shi, minus makes it up for some reason
                // again  ¯\_(ツ)_/¯
                shooter.moveAngleManual(-gamepad2.left_stick_y);

                if (gamepad2.a) {
                    ManualShooting();
                } else {
                    isShooting = false;
                    shooter.StopMotors();
                }
            } else {
                if (gamepad2.a) {
                    LimelightShooting();
                } else {
                    isShooting = false;
                    shooter.StopMotors();
                }
            }


            if (gamepad2.x) {
                shooter.RunBelt(1);
            } else if (gamepad2.b) {
                shooter.RunBelt(-1);
            } else if (!gamepad2.a || isManualMode) {
                shooter.RunBelt(0);
            }


            if(gamepad2.dpadUpWasPressed()){
            increaseLaunchPow();
            }
            if (gamepad2.dpadDownWasPressed()){

                decreaseLaunchPow();
            }







            telemetry.addData("Is manual mode :" ,isManualMode );
            telemetry.addData("Flywheel power", shooter.flywheel.getPower());
            telemetry.addData("Flywheel set speed", manualFlywheelPower);
            telemetry.addData("Angle Manual Pos", shooter.anglePos);

            telemetry.update();
        }
    }


    // Uses Limelight logic
    private void LimelightShooting(){
        isShooting = true;
        shooter.Shoot();
        for (LLResultTypes.FiducialResult res : shooter.GetImage().getFiducialResults()) {
            int id = res.getFiducialId();
            if (id == 20 || id == 24)
                PIDAdjusting(res);
        }
    }



    private void ManualShooting(){
        isShooting = true;
        shooter.FixedShoot(manualFlywheelPower);
    }




    private void PIDAdjusting(LLResultTypes.FiducialResult res){
        double error = res.getTargetXDegrees();
        iSum += error;
        double derError = lastError - error;

        lb.setPower(-((error * p) + (iSum * i) + (derError * d)));
        rb.setPower((error * p) + (iSum * i) + (derError * d));
        lf.setPower(-((error * p) + (iSum * i) + (derError * d)));
        rf.setPower((error * p) + (iSum * i) + (derError * d));

        lastError = error;
    }

    private void ClampSpeed(){
        lStickPosX = Math.clamp(Math.abs(gamepad1.left_stick_x), stickClampMin, stickClampMax)
                * Math.signum(gamepad1.left_stick_x);
        lStickPosY = Math.clamp(Math.abs(gamepad1.left_stick_y), stickClampMin, stickClampMax)
                * Math.signum(gamepad1.left_stick_y);
        SmoothSpeed(lStickPosX, lStickPosY, acceleration, turnAccel);
    }

    private void SmoothSpeed(double posX, double posY, double accelExp, double turnExp){
        lStickPosX = Math.pow(posX, accelExp);
        lStickPosY = Math.pow(posY, accelExp);
        rStickPosX = Math.pow(gamepad1.right_stick_x, turnExp);
    }

    private void InitMotors(){
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
    }

    private void SetDriveDirection(String direction){
        if (direction.equals("forward")){
            lb.setDirection(DcMotorEx.Direction.REVERSE);
            rb.setDirection(DcMotorEx.Direction.FORWARD);
            lf.setDirection(DcMotorEx.Direction.REVERSE);
            rf.setDirection(DcMotorEx.Direction.FORWARD);
            return;
        }

        lb.setDirection(DcMotorEx.Direction.REVERSE);
        rb.setDirection(DcMotorEx.Direction.FORWARD);
        lf.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
    }

    private void SetBrakes(){
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void Drive(){
        ClampSpeed();
        lb.setPower((rStickPosX * -driveSpeed) + (driveSpeed * gamepad1.left_stick_x) + (driveSpeed * gamepad1.left_stick_y));
        rb.setPower((rStickPosX * driveSpeed) + (-driveSpeed * gamepad1.left_stick_x) + (driveSpeed * gamepad1.left_stick_y));
        lf.setPower((rStickPosX * -driveSpeed) + (-driveSpeed * gamepad1.left_stick_x) + (driveSpeed * gamepad1.left_stick_y));
        rf.setPower((rStickPosX * driveSpeed) + (driveSpeed * gamepad1.left_stick_x) + (driveSpeed * gamepad1.left_stick_y));
    }


    private void increaseLaunchPow(){
        manualFlywheelPower+= .1;
    }
    private void decreaseLaunchPow(){
        manualFlywheelPower-= .1;
    }

}