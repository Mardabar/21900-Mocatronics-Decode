package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Map;
import java.util.TreeMap;
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
    public static double kP = 0.005; // was 0.004
    public static double kS = 0.02;
    public static double kV = 0.00045;  //  was 0.00039



    private VoltageSensor battery;

    private TreeMap<Double, Double> distanceToPos = new TreeMap<>();
    private final TreeMap<Double, Double> angleMap = new TreeMap<>();

    private Follower fol;

    private Telemetry telemetry;
    public Limelight3A cam;

    public Servo angleAdjuster;
    public Servo feeder;
    public DcMotorEx belt;
    public DcMotorEx flywheel;

    // Position declaring

    public double anglePos = 0.5;
    public static double openPos = .35;
    public static double closePos = 0;

    // CONSTANTS
    public static double IDLE_VELO = 300;
    private final double MAX_HEIGHT = 1.4;
    public final double OVERSHOOT_VEL_MULT = 2.21;
    public final double OVERSHOOT_ANG_MULT = 1; // was .8
    public double manualServoPos = 0.15;


    // SHOOT VARS

    public double shootAngle;
    public double shootVel;

    public double beltSpeed = 1;
    private enum IntakeState{WAITING, FEED, RECOVER}
    private IntakeState intakeState = IntakeState.WAITING;

    // INIT

    public FeedBackShootSystem(HardwareMap hardwareMap, Telemetry telemetry){

        this.telemetry = telemetry;
        belt = hardwareMap.get(DcMotorEx.class, "belt");
        flywheel = hardwareMap.get(DcMotorEx.class, "cannon");
        angleAdjuster = hardwareMap.get(Servo.class, "angleServo");
        feeder = hardwareMap.get(Servo.class, "feeder");
        battery = hardwareMap.voltageSensor.iterator().next();

        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);

        belt.setDirection(DcMotorEx.Direction.REVERSE);
        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        angleAdjuster.scaleRange(0, 1);

        initDistances();

        fol = Constants.createFollower(hardwareMap);
        fol.setStartingPose(new Pose(27.5, 132)); // was 56, 8

        cam = hardwareMap.get(Limelight3A.class, "limelight");
        cam.pipelineSwitch(0);
        cam.start();

    }

    // PUBLIC METHODS

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
        double power = (ff + fb);

        // Sets power to a value in between 0-1
        flywheel.setPower(Math.clamp(power, -1, 1));
    }


    public void Shoot(){
        LLResult result = cam.getLatestResult();

        if (result != null && result.isValid())
            UpdatePositions(result);


        updateFlywheelControl(shootVel);
        angleAdjuster.setPosition(anglePos);
        fol.update();
    }

    public LLResult GetImage(){
        return cam.getLatestResult();
    }

    private void setShootPos(double dist){
        dist *= 1.2;
        double veloMult = OVERSHOOT_VEL_MULT + (dist * 0.15);

        double rawVel = angleToVel(distToAngle(dist)) * veloMult;

        shootVel = velToTPS(rawVel);
        //shootVel = Math.max(velToTPS(rawVel), IDLE_VELO);

        //anglePos = getServoPosition(dist); // LERP old
        angleInterpolation(dist); // LERP new
    }



    public void angleInterpolation(double distance) {
        Map.Entry<Double, Double> lowEntry = angleMap.floorEntry(distance);
        Map.Entry<Double, Double> highEntry = angleMap.ceilingEntry(distance);

        if (lowEntry == null && highEntry == null) return;

        if (lowEntry == null) {
            anglePos = highEntry.getValue();
        } else if (highEntry == null || lowEntry.getKey().equals(highEntry.getKey())) {
            anglePos = lowEntry.getValue();
        } else {
            double x0 = lowEntry.getKey();
            double y0 = lowEntry.getValue();
            double x1 = highEntry.getKey();
            double y1 = highEntry.getValue();

            anglePos = y0 + (distance - x0) * ((y1 - y0) / (x1 - x0));
        }

        //anglePos += hoodManualAdjustment;
        // Clamp to valid servo range
        anglePos = Math.max(0, Math.min(1, anglePos));
    }


    // LUT servo system
    private void initDistances() {

        // Ok so the first num is the distance in meters and the second num is the position of the servo
        distanceToPos.put(0.8128, 0.0830);
        distanceToPos.put(0.9652, 0.75);
        distanceToPos.put(1.016, 0.820);
        distanceToPos.put(1.2192, 0.083);
        distanceToPos.put(1.2446, 0.15);
        distanceToPos.put(1.4224, 0.15);
        distanceToPos.put(1.8796, 0.195);
        distanceToPos.put(1.6256, 0.106);
        distanceToPos.put(1.8288, 0.186);
        distanceToPos.put(2.9718, 0.15);


        angleMap.put(0.9144, 0.122);
        angleMap.put(1.0922, 0.132);
        angleMap.put(1.1938, 0.130);
        angleMap.put(1.2954, 0.135);
        //angleMap.put(1.4224, 0.150);
        angleMap.put(1.5240, 0.150); // was 0.166
        angleMap.put(1.6256, 0.237);
        angleMap.put(2.0574, 0.248);
        angleMap.put(3.0734, 0.19); // 0.188

        // 36 in, 0.122
        // 43 in, 0.1320
        // 47, 0.130
        // 51, 0.1350
        // 56, 0.1500
        // 60, 0.1660
        //64, 0.2370
        // 81, 0.2480
        //121, 0.1880

    }

    // Here we get the distance using linear interpolation
    public double getServoPosition(double dist){
        Double lowKey = distanceToPos.floorKey(dist);
        Double highKey = distanceToPos.ceilingKey(dist);

        if (highKey == null && lowKey == null)
            return 0.15;

        if (lowKey == null)
            return distanceToPos.get(highKey);

        if (highKey == null)
            return distanceToPos.get(lowKey);

        if(lowKey.equals(highKey))
            return distanceToPos.get(lowKey);

        double t = (dist - lowKey) / (highKey - lowKey);

        double lowVal = distanceToPos.get(lowKey);
        double highVal = distanceToPos.get(highKey);

        return lowVal + t * (highVal - lowVal);
    }

    private void updateVars(LLResultTypes.FiducialResult res){
        double angle = 25.2 + res.getTargetYDegrees();
        double limeDist = (0.646 / Math.tan(Math.toRadians(angle))) + 0.2;

        beltSpeed = 0.2;

        setShootPos(limeDist);

    }

    public void UpdatePositions(LLResult pic){
        if (pic == null) return; // safetey measure here if the cam doesnt get a valid pic
        for (LLResultTypes.FiducialResult res : pic.getFiducialResults()) {
            int id = res.getFiducialId();
            // Only updates if its one of the two april tag values
            if (id == 20 || id == 24)
                updateVars(res);
        }
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

    public void MoveBelt() {
        belt.setPower(beltSpeed);
    }


    public void stopBelt(){
        belt.setPower(0);
    }

    public void StopMotors(){
        flywheel.setPower(0);
        stopBelt();
        //angleAdjuster.setPosition(0.15);
    }

    public void adjustServoManual(boolean up, boolean down) {
        double increment = 0.001; // Tiny steps for precision tuning

        if (up) {
            manualServoPos += increment;
        } else if (down) {
            manualServoPos -= increment;
        }

        manualServoPos = Math.clamp(manualServoPos, 0, 1);

        angleAdjuster.setPosition(manualServoPos);
    }

}