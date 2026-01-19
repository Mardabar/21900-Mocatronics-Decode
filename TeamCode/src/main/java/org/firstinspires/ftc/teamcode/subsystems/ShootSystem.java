package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShootSystem {
    private FlyWheelShooter fws;

    private boolean isFeederUp = false;

    public double anglePos = 0.5;
    private final Telemetry telemetry;
    public Limelight3A cam;

    public DcMotorEx belt;
    public DcMotorEx flywheel;
    private DcMotorEx turretMotor;
    public final Servo angleAdjuster;
    public Servo feeder;
    public static double openPos = .35;
    public static double closePos = 0;

    // CONSTANTS

    private final double OVERSHOOT_VEL_MULT = 2.21;
    private final double OVERSHOOT_ANG_MULT = 1;
    private final double ANGLE_CONST = 2.08833333;
    private final int ELBOW_GEAR_RATIO = 28;
    private final double MAX_HEIGHT = 1.4;

    private final double REST_POS = 0;
    private final double FEED_POS = 0.2;

    // SHOOT VARS

    private double shootAngle;
    public static double shootVel;

    private boolean flipped;

    // INIT

    public ShootSystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        belt = hardwareMap.get(DcMotorEx.class, "belt");
        flywheel = hardwareMap.get(DcMotorEx.class, "cannon");
        angleAdjuster = hardwareMap.get(Servo.class, "angleServo");
        feeder = hardwareMap.get(Servo.class, "feeder");

        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        belt.setDirection(DcMotorEx.Direction.REVERSE);

        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        angleAdjuster.scaleRange(0, 1);

        //angleAdjuster.setPosition(0.15);
//        feeder.setPosition(openPos);

        cam = hardwareMap.get(Limelight3A.class, "limelight");
        cam.pipelineSwitch(0);
        cam.start();

        fws = new FlyWheelShooter(hardwareMap, telemetry);
    }

    // PUBLIC METHODS

    public LLResult GetImage(){
        return cam.getLatestResult();
    }

    public void StopMotors(){
        flywheel.setVelocity(0);
        RunBelt(0);
    }

    public void Shoot(){
        UpdatePositions(cam.getLatestResult());
        double power = fws.GetCalculatedPower(velToTPS(shootVel), flywheel.getVelocity());
        flywheel.setPower(power);
        setShootAngle(shootAngle);

        // Only run the belt if the flywheel is at least 95% of the way to the target
        if (flywheel.getVelocity() > velToTPS(shootVel) * 0.95) {
            RunBelt(1);
        } else {
            RunBelt(0);
        }
    }

    public void moveAngleManual(double joystickInput) {
        double speed = 0.001;
        anglePos += (joystickInput * speed);
        anglePos = Math.clamp(anglePos, 0, 1);
        angleAdjuster.setPosition(anglePos);
    }

    // MAIN METHODS

    private void UpdatePositions(LLResult pic){
        for (LLResultTypes.FiducialResult res : pic.getFiducialResults())
            CheckID(res, res.getFiducialId());
    }

    // keep ts the same cause cam is fine
    private void CheckID(LLResultTypes.FiducialResult res, int id){
        telemetry.addData("id", id);
        telemetry.update();
        boolean idCheck = id == 20 || id == 24;
        if (idCheck)
            UpdateVars(res);
    }

    // keep ts the same cause the cam is fine
    private void UpdateVars(LLResultTypes.FiducialResult res){
        double angle = 25.2 + res.getTargetYDegrees();
        double tagDist = (0.646 / Math.tan(Math.toRadians(angle))) + 0.2;

        if (tagDist - 2 < 0)
            tagDist += (tagDist - 2) * 0.8;
        else
            tagDist += (tagDist - 2) * 0.6;

        setShootPos(tagDist);
    }

    private void setShootPos(double dist){
        /* dist is the total distance the ball will travel until it hits the ground
           it's multiplied by 1.3 because the ball will hit the goal first, so using the
           equation, it'll be about 1 meter high (the height of the goal) when it hit our requested distance
         */
        dist *= 1.3;

        // The angle and velocity are both calculated using the distance we found
        shootAngle = ((distToAngle(dist) * OVERSHOOT_ANG_MULT) - 45);
        shootVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;
    }

    private void setShootAngle(double angle){
        angleAdjuster.setPosition(Math.clamp(angle / 300, 0, 1));
    }

    public void RunBelt(double speed) {
        belt.setPower(speed);
    }


    // CONVERSIONS

    public double distToAngle(double dist){
        return Math.toDegrees(Math.atan(54.88 / (9.8 * dist)));
    }

    // This function translates angle to velocity using the already set maximum height
    public double angleToVel(double angle){
        return Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(angle)), 2));
    }

    // This function translates velocity to motor power specifically for 6000 RPM motors combined with 72 mm Gecko Wheels
    public double velToTPS(double vel) {
        return (vel / (9.6 * Math.PI)) * 2800;
    }


    public void FixedShoot(double flywheelPower) {
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(flywheelPower);
        angleAdjuster.setPosition(anglePos);
    }
}