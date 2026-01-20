package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FeedBackShootSystem {


    // Feedback constants and battery declaring
    public static double kP = 0.001;
    public static double kS = 0.02;
    public static double kV = 0.00043;
    private VoltageSensor battery;

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

        cam = hardwareMap.get(Limelight3A.class, "limelight");
        cam.pipelineSwitch(0);
        cam.start();

    }

    // PUBLIC METHODS

    // Math that we did for the feedback and feedforward





    // NEW SHOOT FUNCTION USING TUNED VALUES
    public void Shoot(){


        //My dumbahh forgot to call the cam update func
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

    // Camera stuff

    public void UpdatePositions(LLResult pic){
        if (pic == null) return; // saftey measure here if the cam doesnt get a valid pic
        for (LLResultTypes.FiducialResult res : pic.getFiducialResults()) {
            int id = res.getFiducialId();
            // Only updates if its one of the two april tag values
            if (id == 20 || id == 24)
                UpdateVars(res);

        }
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
    public void setShootPos(double dist){
        dist *= 1.3;
        shootAngle = ((distToAngle(dist) * OVERSHOOT_ANG_MULT) - 45);

        // This should calc the velocity in m/s if the conversion is correct then apply the overshoot vel which we might not need anymore
        double rawVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;

        // Converts velo to tps
        double calculatedTPS = velToTPS(rawVel);

        // Converts m/s to ticks per second so then the control systems can understand it
        // now checks between which value is higher and then will set to that power
        shootVel = Math.max(rawVel, IDLE_VELO);
    }


    public void setShootAngle(double angle){
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



    // CONVERSIONS

    // keep these as the same, might give off too much power


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