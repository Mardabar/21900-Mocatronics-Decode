package org.firstinspires.ftc.teamcode.UnusedOpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
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


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "CloseRed", group = "autonomous")
public class TempCloseRed extends OpMode{

    // PEDROPATHING VARS
    // These are variables that are neccesary for pedro pathing to work, you need these for the drive motors to work

    private Follower fol;
    private OldBluePaths paths;
    private int pathState; // Current path #

    /** This is a function that lets us change the pathState which we use later in the code,
     * You use this function like this */ // setPathState('NUMBER HERE')*/
    /** Check line 215 for an example */

    private void setPathState(int num){
        pathState = num;
        pathTimer.resetTimer();
    }


    // IGNORE THESE
    private Timer pathTimer, opmodeTimer; // Game timer
//    private int chainNum;
//    private int ballNum = 2;
//    private int shootPos = 1;

    // CAMERA VARS
    /*** WE ARENT USING THESE BECAUSE PEDRO IS MORE CONSISTENT
     private Limelight3A cam;
     private VisionPortal visPort;
     private AprilTagProcessor apTag;
     private LLResultTypes.FiducialResult foundTag;
     private boolean tagFound;
     private final int GPP_ID = 21;
     private final int PGP_ID = 22;
     private final int PPG_ID = 23;
     */


    // FIELD POSITIONS
    /** Pretty self explanatory, use the visualizer to find the paths and positions you want your robot to follow
     * USE CONTROL POINTS!!! */

    private final Pose startPose = new Pose(28, 131, Math.toRadians(144)).mirror(); // STARTING POSITION was 23, 124, 144
    private final Pose preScorePose = new Pose(61, 104, Math.toRadians(145)).mirror(); // PRE-LOAD SCORING POSITION
    private final Pose row1Line = new Pose(50, 84, Math.toRadians(0)).mirror(); // Position
    private final Pose row1Line1CP = new Pose(91,84).mirror(); // CONTROL POINT
    private final Pose row1Grab = new Pose(30, 84, Math.toRadians(0)).mirror(); // Position or 86
    private final Pose row1Score = new Pose(61, 79, Math.toRadians(134)).mirror(); // Scoring
    private final Pose row2Line = new Pose(51, 61.5, Math.toRadians(0)).mirror(); // Position
    private final Pose row2LineCP = new Pose(85, 60).mirror();
    private final Pose row2Grab = new Pose(32, 61.5, Math.toRadians(0)).mirror();
    private final Pose row2Score = new Pose(61, 79, Math.toRadians(134)).mirror();

    private final Pose parkPose = new Pose(50, 72, Math.toRadians(134)).mirror(); // PARKING POSITION



    // MOTORS
    /** Here we initialise all your extra motors and servos
     * Your drive motors should be already set up due to your pedro tuning process */
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
    /** IGNORE THIS FOR RIGHT NOW THIS IS SPECIFIC TO OUR ROBOT FOR ITS SHOOTING FUNCTIONS
     * JUMP TO LINE 121 */
    private final double OVERSHOOT_VEL_MULT = 1.64; // was 1.68
    private final double OVERSHOOT_ANG_MULT = 1;
    private final double ANGLE_CONST = 2.08833333;
    private final int ELBOW_GEAR_RATIO = 4;
    private final double MAX_HEIGHT = 1.4;
    private double shootVel;
    private double shootAngle;


    // INTAKE VARS
    private double elbowSpeed = 0.5;
    private double beltSpeed = 1;


    // SERVO VARS
    private double openPos = 0.53;
    private double feedPos = 0.02;


    // TIMER VARS
    private ElapsedTime feedTimer;
    private double feedDur = 600; // was 400
    private double retDur = 600; // was 1000
    private double beltDur = 500; // was 500, 300
    private ElapsedTime shootTimer;
    private int shootTimerCount = -1;
    private int feeding = 0;
    private int fcount = 0;

    private double fx = 10;
    private double fy = 136.5;



    // PATH CHAINS
    /** Here is where we initialize our path names, this makes writing the robot drive sequence part of the code to be alot simpler to read
     * Use a simple naming method for simplicity
     * For example if you have a pose named "preScore" then you simple add the word 'path' in front
     * preScore -> pathPreScore || row1Line -> pathRow1Line
     *
     * */
    private PathChain pathPreScore, pathRow1Line, pathRow1Grab, pathRow1Score, pathRow2Line, pathRow2Grab, pathRow2Score, pathParkPose;


    @Override
    public void init(){
        // HARDWARE INIT
        /** Initialise your motors and servos like you normally would
         * Make sure to intialize a new data type your 'Follower' and init your starting position
         * Ours is named fol so we use the ''Constants.createFollower(hardwareMap)'' method
         * Examples below */
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

        shootTimer = new ElapsedTime();
        feedTimer = new ElapsedTime();

        // PATH INIT

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();



        // The magic begins
        /** Make sure to set your pathState to 0 or -1, whatever you want your path chain to start with */
        buildPaths();
        setPathState(-1);

    }

    public void loop(){

        // This runs your entire auto process in this function, very important
        fol.update();
        autonomousPathUpdate();


        updatePos();
    }

    public void buildPaths(){

        pathPreScore = fol.pathBuilder()
                .addPath(new BezierLine(startPose, preScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                .build();

        pathRow1Line = fol.pathBuilder()
                .addPath(new BezierLine(preScorePose, row1Line))
                .setLinearHeadingInterpolation(preScorePose.getHeading(), row1Line.getHeading())
                //.setTangentHeadingInterpolation()
                .build();

        pathRow1Grab = fol.pathBuilder()
                .addPath(new BezierLine(row1Line, row1Grab))
                .setLinearHeadingInterpolation(row1Line.getHeading(), row1Grab.getHeading())
                .build();

        pathRow1Score = fol.pathBuilder()
                .addPath(new BezierLine(row1Grab, row1Score))
                .setLinearHeadingInterpolation(row1Grab.getHeading(), row1Score.getHeading())
                .build();

        pathRow2Line = fol.pathBuilder()
                .addPath(new BezierLine(row1Score, row2Line))
                .setLinearHeadingInterpolation(row1Score.getHeading(), row2Line.getHeading())
                .build();

        pathRow2Grab = fol.pathBuilder()
                .addPath(new BezierLine(row2Line, row2Grab))
                .setLinearHeadingInterpolation(row2Line.getHeading(), row2Grab.getHeading())
                .build();

        pathRow2Score = fol.pathBuilder()
                .addPath(new BezierLine(row2Grab, row2Score))
                .setLinearHeadingInterpolation(row2Grab.getHeading(), row2Score.getHeading())
                .build();

        pathParkPose = fol.pathBuilder()
                .addPath(new BezierLine(row2Score, parkPose))
                .setLinearHeadingInterpolation(row2Score.getHeading(), parkPose.getHeading())
                .build();
    }


    public void autonomousPathUpdate(){

        switch (pathState) {
            // Edited so it runs to the first pose and scores preloads
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
                    setShootPos(preScorePose.getX(), preScorePose.getY(), fx, fy); // was 9 , 135
                    setPathState(1);
                }
                break;

            case 1:
                if (!fol.isBusy()) {
                    fol.followPath(pathRow1Grab);
                    fol.setMaxPower(.4);
                    runBelt(beltSpeed);
                    setPathState(2);
                }
                break;

            case 2:
                if (!fol.isBusy()){
                    fol.followPath(pathRow1Score);
                    fol.setMaxPower(1);
                    setShootPos(row1Score.getX(), row1Score.getY(), fx, fy); // was 9 135
                    //runBelt(beltSpeed);
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
                    setShootPos(row2Score.getX(), row2Score.getY(), fx, fy);
                    setPathState(6);
                }
                break;

            case 6:
                if (!fol.isBusy()) {
                    fol.setMaxPower(.25);
                    fol.followPath(pathRow2Grab);
                    setShootPos(row2Score.getX(), row2Score.getY(), fx, fy);
                    runBelt(beltSpeed);
                    //ballNum = 3;
                    setPathState(7);
                }
                break;

            case 7:
                if (!fol.isBusy()){
                    fol.setMaxPower(1);
                    fol.followPath(pathRow2Score);
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
        if (shootTimer.milliseconds() < 9000 && fcount <= 6 ){
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

    private void shoot3(){
        if (shootTimerCount == -1) {
            shootTimer.reset();
            shootTimerCount = 0;
        }

        if (shootTimer.milliseconds() < 1200 && shootTimerCount == 0){
            ls.setVelocity(velToPow(shootVel));
            rs.setVelocity(velToPow(shootVel));
        }
        else if (shootTimerCount == 0){
            shootTimer.reset();
            feedTimer.reset();
            shootTimerCount = 1;
        }
        // Changed the multiplier to 2 because we are grabbing 2 balls instead of 3
        if (shootTimer.milliseconds() < 12000 && fcount <= 9 ){
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
        else if (feedTimer.milliseconds() < beltDur  && feeding == 2) {
            blocker.setPosition(1);
            ascension.setPower(1);
            runBelt(beltSpeed);
        }
        else {
            if (ls.getVelocity() >= velToPow(shootVel) - 15 && rs.getVelocity() >= velToPow(shootVel) - 15) {
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
//        Pose currentPose = fol.getPose();
//
//        double currentX = currentPose.getX();
//        double currentY = currentPose.getY();
//
//        telemetry.addData("Current Path State", pathState);
//        telemetry.addData("X Position", "%.2f", currentX);
//        telemetry.addData("Y Position", "%.2f", currentY);
        telemetry.addData("Right Launch Power", rs.getPower());
        telemetry.addData("Left Launch Power", ls.getPower());
        //telemetry.addData("Pose", fol.getPose());
        telemetry.update();
    }
}