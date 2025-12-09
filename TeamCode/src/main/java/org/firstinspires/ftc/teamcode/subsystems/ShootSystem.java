package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShootSystem {

    private String mode;

    private DcMotorEx ls;
    private DcMotorEx rs;
    private DcMotorEx belt;
    private DcMotorEx elbow;

    private CRServo br;
    private CRServo bl;
    private Servo blocker;
    private CRServo ascension;

    // CONSTANTS

    private final double OVERSHOOT_VEL_MULT = 1.618;
    private final double OVERSHOOT_ANG_MULT = 1;
    private final double ANGLE_CONST = 2.08833333;
    private final int ELBOW_GEAR_RATIO = 28;
    private final double MAX_HEIGHT = 1.4;

    // SHOOT VARS

    public boolean shootPrep;
    public boolean shootReady;
    private double shootAngle;
    private double shootVel;

    // FEEDING VARS

    private double openPos = 0.53;
    private double feedPos = 0.03; // 0.02
    private ElapsedTime blockTimer;
    private ElapsedTime feedTimer;
    private double beltDur = 550;
    private double retDur = 300;
    private double feedDur = 750;
    private int feeding = 2;
    private int fcount;
    private boolean flysSpeedy;

    // INIT

    public ShootSystem(HardwareMap hardwareMap, String mode){
        ls = hardwareMap.get(DcMotorEx.class, "ls");
        rs = hardwareMap.get(DcMotorEx.class, "rs");
        belt = hardwareMap.get(DcMotorEx.class, "belt");
        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        ls.setDirection(DcMotorEx.Direction.FORWARD);
        rs.setDirection(DcMotorEx.Direction.REVERSE);
        belt.setDirection(DcMotorEx.Direction.FORWARD);
        elbow.setDirection(DcMotorEx.Direction.REVERSE);

        br = hardwareMap.get(CRServo.class, "br");
        bl = hardwareMap.get(CRServo.class, "bl");
        blocker = hardwareMap.get(Servo.class, "blocker");
        ascension = hardwareMap.get(CRServo.class, "ascension");

        blocker.scaleRange(feedPos, openPos);
        blocker.setPosition(1);

        blockTimer = new ElapsedTime();
        feedTimer = new ElapsedTime();

        this.mode = mode;
    }

    // MAIN METHODS

    public void initShooting(LLResult pic){
        for (LLResultTypes.FiducialResult res : pic.getFiducialResults()) {
            int id = res.getFiducialId();
            if (id == 20 || id == 24) {
                double angle = 25.2 + res.getTargetYDegrees(); // 25.2
                double tagDist = (0.646 / Math.tan(Math.toRadians(angle)));
                if (tagDist - 2 < 0)
                    tagDist += (tagDist - 2) * 0.28;
                else
                    tagDist += (tagDist - 2) * 0.19;

                setShootPos(tagDist);
                feeding = 2;
                blocker.setPosition(1);
                blockTimer.reset();

                ls.setMotorEnable();
                rs.setMotorEnable();

                shootPrep = true;
            }
        }
    }

    public void shoot(){
        double shootRot = velToRot(shootVel);

        if (ls.getVelocity() < shootRot + 15 && rs.getVelocity() < shootRot + 15)
            setElbowTarget(getAngleEnc());
        ls.setVelocity(shootRot + 250);
        rs.setVelocity(shootRot - 100);

        if (!flysSpeedy && ls.getVelocity() >= (shootRot + 250) && rs.getVelocity() >= (shootRot - 100))
            flysSpeedy = true;
        if (flysSpeedy) {
            feedLauncher();
        }
    }

    public void resetBack(){
        feeding = 2;
        fcount = 0;
        blocker.setPosition(1);
        ascension.setPower(0);
        runBelt(0);
        ls.setPower(0);
        rs.setPower(0);
        ls.setMotorDisable();
        rs.setMotorDisable();

        flysSpeedy = false;
        shootPrep = false;
        shootReady = false;
    }

    private void setShootPos(double dist){
        /* dist is the total distance the ball will travel until it hits the ground
           it's multiplied by 1.3 because the ball will hit the goal first, so using the
           equation, it'll be about 1 meter high (the height of the goal) when it hit our requested distance
         */
        dist *= 1.3;

        // The angle and velocity are both calculated using the distance we found
        shootAngle = ((distToAngle(dist) * OVERSHOOT_ANG_MULT) - 56);
        shootVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;

        shootPrep = false;
        shootReady = true;
    }

    private void setShootPos(double ix, double iy, double fx, double fy){
        /* dist is the total distance the ball will travel until it hits the ground
           it's multiplied by 1.3 because the ball will hit the goal first, so using the
           equation, it'll be about 1 meter high (the height of the goal) when it hit our requested distance
         */
        double dist = (Math.sqrt(Math.pow(fx - ix, 2) + Math.pow(fy - iy, 2)) / 40) * 1.3;

        // The angle and velocity are both calculated using the distance we found
        shootAngle = ((distToAngle(dist) * OVERSHOOT_ANG_MULT) - 56);
        shootVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;
    }

    private void feedLauncher(){
        if (feedTimer.milliseconds() < beltDur && feeding == 0){

            blocker.setPosition(1);
            ascension.setPower(1);
            runBelt(-1);
        }
        else if (feedTimer.milliseconds() < retDur && feeding == 1){
            blocker.setPosition(0);
            ascension.setPower(1);
            runBelt(0);
        }
        else if (feedTimer.milliseconds() < feedDur && feeding == 2) {
            blocker.setPosition(1);
            ascension.setPower(1);
            runBelt(0);
        }
        else {
            if (ls.getVelocity() >= (getShootVel() + 250) - 30 && rs.getVelocity() >= (getShootVel() - 100) - 30) {
                if (feeding == 2)
                    feeding = 0;
                else
                    feeding++;
                fcount++;
            }
            feedTimer.reset();
        }
    }

    private void setElbowTarget(double angle){
        elbow.setTargetPosition((int) angle);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void runBelt(double speed) {
        belt.setPower(speed);
        br.setPower(speed);
        bl.setPower(-speed);
    }

    // GETTERS

    public double getAngleEnc(){
        return angleToEncoder(shootAngle);
    }

    public double getShootVel(){
        return velToRot(shootVel);
    }

    public double getShootAngle(){
        return shootAngle;
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
    public double velToRot(double vel){
        return (vel / (7.2 * Math.PI)) * 2800;
    }

    // This function translates an angle in degrees to an encoder value on 223 RPM motors
    public double angleToEncoder(double angle){
        return angle * ANGLE_CONST * ELBOW_GEAR_RATIO;
    }
}
