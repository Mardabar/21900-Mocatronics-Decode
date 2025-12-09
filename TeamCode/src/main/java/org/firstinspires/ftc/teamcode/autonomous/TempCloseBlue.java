package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.paths.CloseBluePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "CloseBlue", group = "autonomous")
public class TempCloseBlue extends OpMode{

    // PEDROPATHING VARS

    private Follower fol;

    private CloseBluePaths paths;

    private Timer pathTimer, actionTimer, opmodeTimer; // Game timer
    private int pathState; // Current path #
    private int chainNum;
    private int ballNum = 2;
    private int shootPos = 1;

    private final int GPP_ID = 21;
    private final int PGP_ID = 22;
    private final int PPG_ID = 23;


    // POSITIONS

    private final Pose startPose = new Pose(28, 131, Math.toRadians(144)); // Starting Point was 23, 124 , 144
    private final Pose preScorePose = new Pose(60, 104, Math.toRadians(146)); // PRE-LOAD SCORING POSITION
    private final Pose lineRow1 = new Pose(44.5, 84, Math.toRadians(0)); // Position
    private final Pose lineRow1CP = new Pose(91,84); // Control Point
    private final Pose grabRow1 = new Pose(30, 84, Math.toRadians(0)); // Position
    private final Pose scoreRow1 = new Pose(61, 78, Math.toRadians(132)); // Scoring Position
    private final Pose lineRow2 = new Pose(47, 60, Math.toRadians(0)); // Position
    private final Pose row2LineCP = new Pose(85, 60); // Control Point
    private final Pose grabRow2 = new Pose(30, 59.5, Math.toRadians(0)); // Position
    private final Pose scoreRow2 = new Pose(61, 78, Math.toRadians(132)); // Scoring Position

    private final Pose parkPose = new Pose(50, 72, Math.toRadians(132)); // Parking Position



    // MOTORS
    private DcMotorEx ls;
    private DcMotorEx rs;
    private DcMotor belt;
    private DcMotor elbow;

    // SERVOS
    private CRServo bl;
    private CRServo br;
    private CRServo ascension;
    private Servo blocker;


    // SHOOTING VARS
    private final double OVERSHOOT_VEL_MULT = 1.68; // was 1.68
    private final double OVERSHOOT_ANG_MULT = 1;
    private final double ANGLE_CONST = 2.08833333;
    private final int ELBOW_GEAR_RATIO = 4;
    private final double MAX_HEIGHT = 1.4;
    private double shootVel;
    private double shootAngle;

    private double fx = 10; // was 9
    private double fy = 136.5; // was 135


    // INTAKE VARS
    private double elbowSpeed = 0.5;
    private double beltSpeed = 1;


    // SERVO VARS
    private double openPos = 0.53;
    private double feedPos = 0.02;


    // TIMER VARS
    private ElapsedTime feedTimer;
    private double feedDur = 450; // was 450
    private double retDur = 600; // was 1000
    private double beltDur = 500; // was 500, 300
    private int feeding = 0;
    private int fcount = 0;


    // PATH CHAINS
    private PathChain pathPreScore, pathRow1Line, pathGrabRow1, pathScoreRow1, pathRow2Line, pathGrabRow2, pathScoreRow2, pathParkPose;


    // OTHER VARS
    private ElapsedTime timer;
    private double dur;

    private ElapsedTime shootTimer;
    private int shootTimerCount = -1;

    @Override
    public void init(){

        paths = new CloseBluePaths(fol);

        // HARDWARE INIT
        fol = Constants.createFollower(hardwareMap);
        fol.setStartingPose(startPose);

        ls = hardwareMap.get(DcMotorEx.class, "ls");
        rs = hardwareMap.get(DcMotorEx.class, "rs");
        belt = hardwareMap.get(DcMotor.class, "belt");
        elbow = hardwareMap.get(DcMotor.class, "elbow");

        bl = hardwareMap.get(CRServo.class, "bl");
        br = hardwareMap.get(CRServo.class, "br");
        ascension = hardwareMap.get(CRServo.class, "ascension");
        blocker = hardwareMap.get(Servo.class, "blocker");

        MotorConfigurationType configRs = rs.getMotorType().clone();
        configRs.setAchieveableMaxRPMFraction(1.0);
        rs.setMotorType(configRs);
        rs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorConfigurationType configLs = ls.getMotorType().clone();
        configRs.setAchieveableMaxRPMFraction(1.0);
        ls.setMotorType(configLs);
        ls.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);
        belt.setDirection(DcMotorSimple.Direction.FORWARD);
        elbow.setDirection(DcMotor.Direction.REVERSE);

        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setPower(elbowSpeed);

        blocker.scaleRange(feedPos, openPos);
        blocker.setPosition(1);
        //blocker.setPosition(0.63);


        // TIMER INIT

        timer = new ElapsedTime();
        shootTimer = new ElapsedTime();
        feedTimer = new ElapsedTime();

        // PATH INIT

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        // CAMERA INIT

    /*    tagFound = false;

        cam = hardwareMap.get(Limelight3A.class, "limelight");
        cam.pipelineSwitch(0);

        cam.start(); */

        // The magic begins

        setPathState(-1);

    }

    public void loop(){
        fol.update();
        autonomousPathUpdate();

        // This stores the ending position of the bot at the end of auto
        Pose finalPose = fol.getPose();

        // Not sure if this is in the right spot :skull:
        // Its either inside the loop or outside but outside prolly wouldnt make sense
        updatePos();
    }

    public void autonomousPathUpdate(){

        switch (pathState) {
            case -1:
                if (!fol.isBusy()){
                    fol.followPath(pathPreScore);
                    setShootPos(preScorePose.getX(), preScorePose.getY(), fx, fy);
                    runBelt(0);
                    setPathState(-2);
                }
                break;

            case -2:
                if (!fol.isBusy()){
                    setPathState(-12);
                }
                break;

            case -12:
                if (shootTimerCount != 2)
                    shoot();
                else {
                    shootTimerCount = -1;
                    setPathState(0);
                }
                break;

            case 0:
                if (!fol.isBusy() && pathState == 0) {
                    fol.followPath(pathRow1Line);
                    setShootPos(preScorePose.getX(), preScorePose.getY(), fx, fy);
                    setPathState(1);
                }
                break;

            case 1:
                if (!fol.isBusy()) {
                    fol.followPath(pathGrabRow1);
                    fol.setMaxPower(.4);
                    runBelt(beltSpeed);
                    setPathState(2);
                }
                break;

            case 2:
                if (!fol.isBusy()){
                    fol.followPath(pathScoreRow1);
                    fol.setMaxPower(1);
                    setShootPos(scoreRow1.getX(), scoreRow1.getY(), fx, fy);
                    runBelt(0);
                    setPathState(3);
                }
                break;

            case 3:
                if (!fol.isBusy()){
                    setPathState(4);
                }
                break;

            case 4:
                if (shootTimerCount != 2)
                    shoot();
                else {
                    shootTimerCount = -1;
                    setPathState(5);
                }
                break;

            case 5:
                if (!fol.isBusy() && pathState == 5){
                    fol.followPath(pathRow2Line);
                    runBelt(beltSpeed);
                    setShootPos(scoreRow2.getX(), scoreRow2.getY(), fx, fy);
                    setPathState(6);
                }
                break;

            case 6:
                if (!fol.isBusy()) {
                    fol.setMaxPower(.25);
                    fol.followPath(pathGrabRow2);
                    runBelt(beltSpeed);
                    //ballNum = 3;
                    setPathState(7);
                }
                break;

            case 7:
                if (!fol.isBusy()){
                    fol.setMaxPower(1);
                    fol.followPath(pathScoreRow2);
                    setShootPos(scoreRow2.getX(), scoreRow2.getY(), fx, fy);
                    runBelt(0);
                    setPathState(8);
                }
                break;

            case 8:
                if (!fol.isBusy()){
                    setPathState(9);
                }
                break;

            case 9:
                if (shootTimerCount != 2)
                    shoot();
                else {
                    shootTimerCount = -1;
                    setPathState(10);
                }
                break;

            case 10:
                if (!fol.isBusy() && pathState == 10){
                    fol.followPath(pathParkPose);
                    setPathState(11);
                }
                break;

            case 11:
                break;
        }
    }


    private void setPathState(int num){
        pathState = num;
        pathTimer.resetTimer();
    }


    // This method sets the speed of the shooter motors and the angle of the shooting posit
    private void setShootPos(double ix, double iy, double fx, double fy){
    /* dist is the total distance the ball will travel until it hits the ground
       It's divided by 40 to turn the field units into meters
       Then, it's multiplied by 1.3 because the ball will hit the goal first, so using
       equation, it'll be about 1 meter high (the height of the goal) when it hit our r
     */
        double dist = (Math.sqrt(Math.pow(fx - ix, 2) + Math.pow(fy - iy, 2)) / 40) * 1.3;

        // The angle and velocity are both calculated using the distance we found
        shootAngle = ((distToAngle(dist) * OVERSHOOT_ANG_MULT) - 53.5);
        shootVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;

        telemetry.addData("Distance", dist);
        telemetry.addData("Angle", shootAngle);
        telemetry.addData("Real Angle", distToAngle(dist));
        telemetry.addData("Velocity", shootVel);
        telemetry.addData("Real Velocity", angleToVel(distToAngle(dist)));
        telemetry.update();

        setElbowTarget(angleToEncoder(shootAngle));
    }

    private double distToAngle(double dist){
        return Math.toDegrees(Math.atan(54.88 / (9.8 * dist)));
    }

    // This function translates angle to velocity using the already set maximum height
    private double angleToVel(double angle){
        return Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(angle)), 2));
    }

    // This function translates velocity to motor power specifically for 6000 RPM motors co
    private double velToPow(double vel){
        return (vel / (7.2 * Math.PI)) * 2800;
    }


    // This function translates an angle in degrees to an encoder value on 223 RPM motors
    private double angleToEncoder(double angle){
        return angle * ANGLE_CONST * ELBOW_GEAR_RATIO;
    }

    private void setElbowTarget(double angle){
        elbow.setTargetPosition((int) angleToEncoder(angle));
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // shootTimerCount has to = -1 for it to reset to 0 here to move on in the function
    private void shoot(){
        if (shootTimerCount == -1) {
            shootTimer.reset();
            shootTimerCount = 0;
        }

        if (shootTimer.milliseconds() < 1200 && shootTimerCount == 0 /* && shootPos == 1*/){
            ls.setVelocity(velToPow(shootVel));
            rs.setVelocity(velToPow(shootVel));
        }
        else if (shootTimerCount == 0){
            shootTimer.reset();
            feedTimer.reset();
            shootTimerCount = 1;
        }
        // Changed the multiplier to 2 because we are grabbing 2 balls instead of 3
        if (shootTimer.milliseconds() < 7000 && fcount <= 6 && shootTimerCount == 1){
            feedLauncher();
        }
        else if (shootTimerCount == 1)
            shootTimerCount = 2;

        if (shootTimerCount == 2){
            ls.setVelocity(0);
            rs.setVelocity(0);
            feeding = 2;
            fcount = 0;
            ascension.setPower(0);
            runBelt(0);
            blocker.setPosition(1);
        }
    }

    private void runBelt(double speed){
        belt.setPower(-speed);
        br.setPower(-speed);
        bl.setPower(speed);
    }

    private void feedLauncher(){
        if (feedTimer.milliseconds() < feedDur && feeding == 0){
            blocker.setPosition(0);
            ascension.setPower(1);
            runBelt(0);
        }
        else if (feedTimer.milliseconds() < retDur && feeding == 1){
            blocker.setPosition(1);
        }
        else if (feedTimer.milliseconds() < beltDur && feeding == 2) {
            blocker.setPosition(1);
            ascension.setPower(1);
            runBelt(beltSpeed);
        }
        else {
            if (ls.getVelocity() >= velToPow(shootVel) && rs.getVelocity() >= velToPow(shootVel)) {
                if (feeding == 2)
                    feeding = 0;
                else
                    feeding++;
                fcount++;
            }
            feedTimer.reset();
        }
    }

    // Updates the pos to the station
    public void updatePos(){
        fol.update();

        // Get the position of the robot
        Pose currentPose = fol.getPose();

        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        telemetry.addData("Current Path State", pathState);
        telemetry.addData("X Position", "%.2f", currentX);
        telemetry.addData("Y Position", "%.2f", currentY);
        telemetry.addData("Right Launch Power", rs.getPower());
        telemetry.addData("Left Launch Power", ls.getPower());
        telemetry.update();
    }
}