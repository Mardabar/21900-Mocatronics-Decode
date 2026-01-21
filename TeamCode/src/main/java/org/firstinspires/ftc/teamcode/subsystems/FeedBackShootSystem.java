package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FeedBackShootSystem {
    // Feedback constants and battery declaring
    public static double kP = 0.001;
    public static double kS = 0.02;
    public static double kV = 0.00043;
    private VoltageSensor battery;

    private Follower fol;
    private Pose redPos = new Pose(132, 135);
    private Pose bluePos = new Pose(12, 135);

    private Telemetry telemetry;
    public Limelight3A cam;

    public DcMotorEx belt;
    public DcMotorEx flywheel;
    public Servo angleAdjuster;

    public Servo feeder;

    // Position declaring

    public static double openPos = .35;
    public double anglePos = 0.5;

    // CONSTANTS

    public final double OVERSHOOT_VEL_MULT = 2.21;
    public final double OVERSHOOT_ANG_MULT = 1;

    private final double MAX_HEIGHT = 1.4;
    public static double IDLE_VELO = 800;


    // SHOOT VARS

    public double shootAngle;
    public double shootVel;

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

        fol = Constants.createFollower(hardwareMap);
        fol.setStartingPose(new Pose(56, 8));

        cam = hardwareMap.get(Limelight3A.class, "limelight");
        cam.pipelineSwitch(0);
        cam.start();
    }

    // PUBLIC METHODS

    public void Shoot(){
        setShootAngle(shootAngle);
        UpdatePositions(cam.getLatestResult());
        updateFlywheelControl(shootVel);
        RunBelt(1);

        fol.update();
    }

    public void moveAngleManual(double joystickInput) {
        double speed = 0.001;
        anglePos += (joystickInput * speed);
        anglePos = Math.clamp(anglePos, 0, 1);
        angleAdjuster.setPosition(anglePos);
    }

    public LLResult GetImage(){
        return cam.getLatestResult();
    }

    // MAIN METHODS

    public void UpdatePositions(LLResult pic){
        if (pic == null) return; // safetey measure here if the cam doesnt get a valid pic
        for (LLResultTypes.FiducialResult res : pic.getFiducialResults()) {
            int id = res.getFiducialId();
            // Only updates if its one of the two april tag values
            if (id == 20 || id == 24)
                UpdateVars(res);
        }
    }

    private void UpdatePods(LLResultTypes.FiducialResult res){
        double tagDist = 0;
        if (res.getFiducialId() == 24)
            tagDist = Math.sqrt(Math.pow(redPos.getX() - fol.getPose().getX(), 2)
                    + Math.pow(redPos.getY() - fol.getPose().getY(), 2));
        else if (res.getFiducialId() == 20)
            tagDist = Math.sqrt(Math.pow(bluePos.getX() - fol.getPose().getX(), 2)
                    + Math.pow(bluePos.getY() - fol.getPose().getY(), 2));
        tagDist *= 0.0254;

        double angle = 25.2 + res.getTargetYDegrees();
        double limeDist = (0.646 / Math.tan(Math.toRadians(angle))) + 0.2;

        /*if (limeDist - 2 < 0)
            limeDist += (limeDist - 2) * 0.8;
        else
            limeDist += (limeDist - 2) * 0.6;*/

        if (Math.abs(tagDist - limeDist) <= 0.5) {
            setShootPos(tagDist);
            telemetry.addData("Last Tag Dist", tagDist);
            telemetry.update();
            return;
        }

        setShootPos(limeDist);
    }

    public void UpdateVars(LLResultTypes.FiducialResult res){
        double angle = 25.2 + res.getTargetYDegrees();
        double tagDist = (0.646 / Math.tan(Math.toRadians(angle))) + 0.2;

        if (tagDist - 2 < 0)
            tagDist += (tagDist - 2) * 0.8;
        else
            tagDist += (tagDist - 2) * 0.6;

        setShootPos(tagDist);

        // adds data about last snapshot cam took
        telemetry.addData("Last Tag Dist", tagDist);
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
        // now checks between which value is higher and then will set to that power
        shootVel = Math.max(velToTPS(rawVel), IDLE_VELO);
    }

    private void setShootAngle(double angle){
        angleAdjuster.setPosition(Math.clamp(angle / 300, 0, 1));
    }

    // method that should make the motor spin at all times at a low speed
    // checks if there is a valid cam pic first
    public void spinUpWhileDriving(){
        LLResult result = cam.getLatestResult();

        if (result != null && result.isValid()){
            UpdatePositions(result);
        } else {
            // if no tag is seen then automatically sets power to the idle value
            shootVel = IDLE_VELO;
        }

        // then just keeps that power constant
        updateFlywheelControl(shootVel);
        setShootAngle(shootAngle);
    }

    // Function that adjusts the flywheel motor power based on battery voltage and current motor velocity in ticks per second
    public void updateFlywheelControl(double targetTPS) {
        double currentTPS = flywheel.getVelocity();
        double currentVoltage = battery.getVoltage();

        // finds feedforward amount based off current velocity from flywheel
        double ff = (kV * targetTPS) + (kS * Math.signum(targetTPS));

        // error correction for if the flywheel overshoots or undershoots its target velocity
        double error = targetTPS - currentTPS;
        double fb = kP * error;

        // voltage compensation for inconsistent battery
        double power = (ff + fb) * (12.0 / currentVoltage);

        // Sets power to a value in between 0-1
        flywheel.setPower(Math.clamp(power, -1, 1));
    }

    // CONVERSIONS

    public double distToAngle(double dist){
        return Math.toDegrees(Math.atan(54.88 / (9.8 * dist)));
    }

    // This function translates angle to velocity using the already set maximum height
    public double angleToVel(double angle){
        return Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(angle)), 2));
    }

    public double velToTPS(double vel) {
        return (vel / (9.6 * Math.PI)) * 2800;
    }

    public void RunBelt(double speed) {
        belt.setPower(speed);
    }

    public void stopBelt(){
        belt.setPower(0);
    }

    public void StopMotors(){
        flywheel.setVelocity(0);
        RunBelt(0);
    }
}