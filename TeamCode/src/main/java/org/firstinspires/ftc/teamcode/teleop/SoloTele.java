//package org.firstinspires.ftc.teamcode.teleop;
//
//import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.openPos;
//import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.closePos;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.subsystems.FeedBackShootSystem;
//
//@TeleOp(name = "Solo Tele")
//public class SoloTele extends LinearOpMode {
//    private FeedBackShootSystem shooter;
//    private boolean isShooting;
//
//    // Drive Vars
//
//    private DcMotorEx lb;
//    private DcMotorEx rb;
//    private DcMotorEx lf;
//    private DcMotorEx rf;
//
//    private final double driveSpeed = 1;
//    private final double acceleration = 4;
//    private final double turnAccel = 4;
//
//    private final double p = 0.021, i = 0.00001, d = 0.00011;
//    private double lastError;
//    private double iSum;
//
//
//    private double lStickPosX;
//    private double lStickPosY;
//    private double rStickPosX;
//    private final double stickClampMin = 0.3;
//    private final double stickClampMax = 1;
//
//    @Override
//    public void runOpMode(){ // INITIALIZATION
//        InitMotors();
//        SetDriveDirection("forward");
//        SetBrakes();
//        shooter = new FeedBackShootSystem(hardwareMap, telemetry);
//
//        waitForStart();
//        while (opModeIsActive())
//            Running();
//    }
//
//    private void Running(){
//        if (!isShooting || gamepad1.left_bumper)
//            Drive();
//
//        if (gamepad1.yWasPressed()){
//            shooter.feeder.setPosition(closePos);
//        } else if (gamepad1.yWasReleased())
//            shooter.feeder.setPosition(openPos);
//
//        if (gamepad1.a)
//            Shooting();
//        else if (gamepad1.aWasReleased()) {
//            isShooting = false;
//            iSum = 0;
//            shooter.StopMotors();
//        } else {
//            if (gamepad1.x)
//                shooter.RunBelt(1);
//            else if (gamepad1.b)
//                shooter.RunBelt(-1);
//            else
//                shooter.RunBelt(0);
//
//            //shooter.spinUpWhileDriving();
//        }
//    }
//
//    private void Drive(){
//        ClampSpeed();
//        lb.setPower((rStickPosX * -driveSpeed) + (driveSpeed * lStickPosX) + (driveSpeed * lStickPosY));
//        rb.setPower((rStickPosX * driveSpeed) + (-driveSpeed * lStickPosX) + (driveSpeed * lStickPosY));
//        lf.setPower((rStickPosX * -driveSpeed) + (-driveSpeed * lStickPosX) + (driveSpeed * lStickPosY));
//        rf.setPower((rStickPosX * driveSpeed) + (driveSpeed * lStickPosX) + (driveSpeed * lStickPosY));
//    }
//
//    private void Shooting(){
//        isShooting = true;
//        shooter.Shoot();
//        for (LLResultTypes.FiducialResult res : shooter.GetImage().getFiducialResults()) {
//            int id = res.getFiducialId();
//            if (id == 20 || id == 24)
//                PIDAdjusting(res);
//        }
//    }
//
//    private void PIDAdjusting(LLResultTypes.FiducialResult res){
//        double error = res.getTargetXDegrees();
//        iSum += error;
//        double derError = lastError - error;
//
//        lb.setPower((error * p) + (iSum * i) + (derError * d));
//        rb.setPower(-((error * p) + (iSum * i) + (derError * d)));
//        lf.setPower((error * p) + (iSum * i) + (derError * d));
//        rf.setPower(-((error * p) + (iSum * i) + (derError * d)));
//
//        lastError = error;
//    }
//
//    private void ClampSpeed(){
//        lStickPosX = Math.clamp(Math.abs(gamepad1.left_stick_x), stickClampMin, stickClampMax)
//                * Math.signum(gamepad1.left_stick_x);
//        lStickPosY = Math.clamp(Math.abs(gamepad1.left_stick_y), stickClampMin, stickClampMax)
//                * Math.signum(gamepad1.left_stick_y);
//        SmoothSpeed(lStickPosX, lStickPosY, acceleration, turnAccel);
//    }
//
//    private void SmoothSpeed(double posX, double posY, double accelExp, double turnExp){
//        lStickPosX = Math.pow(posX, accelExp) * Math.signum(gamepad1.left_stick_x);
//        lStickPosY = Math.pow(posY, accelExp) * Math.signum(gamepad1.left_stick_y);
//        rStickPosX = Math.pow(gamepad1.right_stick_x, turnExp) * Math.signum(gamepad1.right_stick_x);
//    }
//
//    private void InitMotors(){
//        lb = hardwareMap.get(DcMotorEx.class, "lb");
//        rb = hardwareMap.get(DcMotorEx.class, "rb");
//        lf = hardwareMap.get(DcMotorEx.class, "lf");
//        rf = hardwareMap.get(DcMotorEx.class, "rf");
//    }
//
//    private void SetDriveDirection(String direction){
//        if (direction.equals("forward")){
//            lb.setDirection(DcMotorEx.Direction.REVERSE);
//            rb.setDirection(DcMotorEx.Direction.FORWARD);
//            lf.setDirection(DcMotorEx.Direction.REVERSE);
//            rf.setDirection(DcMotorEx.Direction.FORWARD);
//            return;
//        }
//
//        lb.setDirection(DcMotorEx.Direction.REVERSE);
//        rb.setDirection(DcMotorEx.Direction.FORWARD);
//        lf.setDirection(DcMotorEx.Direction.REVERSE);
//        rf.setDirection(DcMotorEx.Direction.FORWARD);
//    }
//
//    private void SetBrakes(){
//        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//}
