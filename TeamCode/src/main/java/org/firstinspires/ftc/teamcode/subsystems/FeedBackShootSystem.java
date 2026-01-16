package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FeedBackShootSystem {

    private boolean isFeederUp = false;

    public static double kP = 0.001;
    public static double kS = 0.02;
    public static double kV = 0.00043;

    private VoltageSensor battery;

    public double anglePos = 0.5;
    private Telemetry telemetry;
    public Limelight3A cam;

    public DcMotorEx belt;
    public DcMotorEx flywheel;
    private DcMotorEx turretMotor;
    public Servo angleAdjuster;

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

    public FeedBackShootSystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        belt = hardwareMap.get(DcMotorEx.class, "belt");
        flywheel = hardwareMap.get(DcMotorEx.class, "cannon");
        angleAdjuster = hardwareMap.get(Servo.class, "angleServo");
        feeder = hardwareMap.get(Servo.class, "feeder");
        battery = hardwareMap.voltageSensor.iterator().next();





        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);

        belt.setDirection(DcMotorEx.Direction.REVERSE);

        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



        angleAdjuster.scaleRange(0, 1);

        angleAdjuster.setPosition(0.15);
//        feeder.setPosition(openPos);

        cam = hardwareMap.get(Limelight3A.class, "limelight");
        cam.pipelineSwitch(0);
        cam.start();



    }

    // PUBLIC METHODS


    // dont use rn

    public LLResult GetImage(){
        return cam.getLatestResult();
    }

    public void StopMotors(){
        flywheel.setVelocity(0);
        RunBelt(0);
    }


    // Math that we did for the feedback and feedforward system
    // Found alot of ts online and will probably have to tweak and change but the impl is there
    public void updateFlywheelControl(double targetTPS) {
        double currentTPS = flywheel.getVelocity();
        double currentVoltage = battery.getVoltage();

        // finds feedforward amount
        double ff = (kV * targetTPS) + (kS * Math.signum(targetTPS));

        // error correction
        double error = targetTPS - currentTPS;
        double fb = kP * error;

        // voltage compensation
        double power = (ff + fb) * (12.0 / currentVoltage);

        flywheel.setPower(Math.clamp(power, -1, 1));
    }


    // NEW SHOOT FUNCTION USING TUNED VALUES
    public void Shoot(){
        setShootAngle(shootAngle);

        UpdatePositions(cam.getLatestResult());

        // shootVel is the target speed
        updateFlywheelControl(shootVel);



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

    private void CheckID(LLResultTypes.FiducialResult res, int id){
        boolean idCheck = id == 20 || id == 24;
        if (idCheck)
            UpdateVars(res);
    }

    //
    private void UpdateVars(LLResultTypes.FiducialResult res){
        double angle = 25.2 + res.getTargetYDegrees();
        double tagDist = (0.646 / Math.tan(Math.toRadians(angle))) + 0.2;

        if (tagDist - 2 < 0)
            tagDist += (tagDist - 2) * 0.8;
        else
            tagDist += (tagDist - 2) * 0.6;

        setShootPos(tagDist);
    }


    // NEW SET SHOOT POS METHOD
    // uses some of ts
    // got these methods from doing math and finding conversions of stuff
    private void setShootPos(double dist){
        dist *= 1.3;
        shootAngle = ((distToAngle(dist) * OVERSHOOT_ANG_MULT) - 45);

        // This should calc the velocity in m/s if the conversion is correct then apply the overshoot vel which we might not need anymore
        double rawVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;

        // Converts m/s to ticks per second so then the control systems can understand it
        // then sets motor power to that i think maybe
        shootVel = velToTPS(rawVel);
    }


    // yea leif can mess w this
    private void setShootAngle(double angle){
        angleAdjuster.setPosition(Math.clamp(angle / 300, 0, 1));
    }

    public void RunBelt(double speed) {
        belt.setPower(speed);
    }

    // CONVERSIONS
    // keep these as the same, might give off too much power but thats ok

    public double distToAngle(double dist){
        return Math.toDegrees(Math.atan(54.88 / (9.8 * dist)));
    }

    // This function translates angle to velocity using the already set maximum height
    public double angleToVel(double angle){
        return Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(angle)), 2));
    }

    // This function translates velocity to motor power for 6000 RPM motors combined with 72 mm Gecko Wheels

    public double velToTPS(double vel) {
        return (vel / (9.6 * Math.PI)) * 2800;
    }


    public void FixedShoot(double flywheelPower) {
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(flywheelPower);
        angleAdjuster.setPosition(anglePos);
    }
}