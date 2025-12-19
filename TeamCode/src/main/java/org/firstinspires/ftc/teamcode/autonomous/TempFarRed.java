package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "FarRed", group = "autonomous")
public class TempFarRed extends OpMode{

    // PEDROPATHING VARS

    private Follower fol;

    private Timer pathTimer, actionTimer, opmodeTimer; // Game timer
    private int pathState; // Current path #
    private int chainNum;

    // POSITIONS

    private final Pose Start = new Pose(56, 8.6, Math.toRadians(90)).mirror(); // STARTING POSITION
    private final Pose preScorePose = new Pose(60, 22, Math.toRadians(115)).mirror(); // PRE-LOAD SCORING POSITION
    private final Pose row1Line = new Pose(45, 35.5, Math.toRadians(0)).mirror(); // POSITION
    private final Pose row1Grab = new Pose(30, 35.5, Math.toRadians(0)).mirror(); // POSITION
    private final Pose row1Score = new Pose(60, 75, Math.toRadians(131)).mirror(); // POSITION
    private final Pose row1ScoreCP = new Pose(60, 34.4, Math.toRadians(131)).mirror(); // CONTROL POINT
    private final Pose row2Line = new Pose(45, 60, Math.toRadians(0)).mirror(); // POSITION
    private final Pose row2Grab = new Pose(31, 60, Math.toRadians(0)).mirror(); // POSITION
    private final Pose row2Score = new Pose(60, 75, Math.toRadians(139)).mirror(); // POSITION
    private final Pose Grab3Set = new Pose(45, 84, Math.toRadians(0)).mirror(); // POSITION
    private final Pose Grab3 = new Pose(31, 84, Math.toRadians(0)).mirror(); // POSITION
    private final Pose Score3 = new Pose(60, 75, Math.toRadians(136)).mirror(); // POSITION
    private final Pose parkPose = new Pose(50, 65, Math.toRadians(139)).mirror(); // PARKING POSITION

    // SHOOTING VARS

    private DcMotorEx ls;
    private DcMotorEx rs;
    private DcMotor belt;
    private DcMotor elbow;

    private final double OVERSHOOT_VEL_MULT = 1.68;
    private final double OVERSHOOT_ANG_MULT = 1;
    private final double ANGLE_CONST = 2.08833333;
    private final int ELBOW_GEAR_RATIO = 4;
    private final double MAX_HEIGHT = 1.4;

    private double shootVel;
    private double shootAngle;
    private double elbowSpeed = 0.5;

    private CRServo bl;
    private CRServo br;
    private CRServo ascension;
    private double beltSpeed = 1;

    private Servo blocker;
    private double openPos = 0.53;
    private double feedPos = 0.02;
    private ElapsedTime feedTimer;
    private double feedDur = 450; // was 400
    private double retDur = 600; // was 1000
    private double beltDur = 800; // was 500, 300
    private int feeding = 0;
    private int fcount = 0;

    private double fx = 10;
    private double fy = 136.5;


    // PATH CHAIN

    private PathChain pathPreScore, pathRow1Line, pathRow1Grab, pathRow1Score, pathRow2Line, pathRow2Grab, pathRow2Score, pathParkPose;

    // OTHER VARS

    private ElapsedTime timer;
    private double dur;
    private int timerCount = -1;

    private ElapsedTime shootTimer;
    private int shootTimerCount = -1;

    @Override
    public void init(){
        // HARDWARE INIT
        fol = Constants.createFollower(hardwareMap);
        fol.setStartingPose(Start);


        ls = hardwareMap.get(DcMotorEx.class, "ls");
        rs = hardwareMap.get(DcMotorEx.class, "rs");
        belt = hardwareMap.get(DcMotor.class, "belt");
        elbow = hardwareMap.get(DcMotor.class, "elbow");

        bl = hardwareMap.get(CRServo.class, "bl");
        br = hardwareMap.get(CRServo.class, "br");
        ascension = hardwareMap.get(CRServo.class, "ascension");
        blocker = hardwareMap.get(Servo.class, "blocker");

        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);
        belt.setDirection(DcMotorSimple.Direction.FORWARD);
        elbow.setDirection(DcMotor.Direction.REVERSE);

        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setPower(elbowSpeed);

        blocker.scaleRange(feedPos, openPos);

        // TIMER INIT

        timer = new ElapsedTime();
        shootTimer = new ElapsedTime();
        feedTimer = new ElapsedTime();

        // PATH INIT

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        buildPaths(0);

        setPathState(-1);
    }

    public void loop(){
        fol.update();
        autonomousPathUpdate();

        // This stores the ending position of the bot at the end of auto

        // Not sure if this is in the right spot :skull:
        // Its either inside the loop or outside but outside prolly wouldnt make sense
        updatePos();
    }

    public void buildPaths(int obNum) {
        pathPreScore = fol.pathBuilder()
                .addPath(new BezierLine(Start, preScorePose))
                .setLinearHeadingInterpolation(Start.getHeading(), preScorePose.getHeading())
                .setBrakingStrength(4)
                .build();

        pathRow1Line = fol.pathBuilder()
                .addPath(new BezierLine(preScorePose, row1Line))
                .setLinearHeadingInterpolation(preScorePose.getHeading(), row1Line.getHeading())
                .build();

        pathRow1Grab = fol.pathBuilder()
                .addPath(new BezierLine(row1Line, row1Grab))
                .setLinearHeadingInterpolation(row1Line.getHeading(), row1Grab.getHeading())
                .build();

        pathRow1Score = fol.pathBuilder()
                .addPath(new BezierCurve(row1Grab, row1ScoreCP, row1Score))
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

//        pathGrab3Set = fol.pathBuilder()
//                .addPath(new BezierLine(Score2, Grab3Set))
//                .setLinearHeadingInterpolation(Score2.getHeading(), Grab3Set.getHeading())
//                .build();
//
//        pathGrab3 = fol.pathBuilder()
//                .addPath(new BezierLine(Grab3Set, Grab3))
//                .setLinearHeadingInterpolation(Grab3Set.getHeading(), Grab3.getHeading())
//                .build();
//
//        pathScore3 = fol.pathBuilder()
//                .addPath(new BezierLine(Grab3, Score3))
//                .setLinearHeadingInterpolation(Grab3.getHeading(), Score3.getHeading())
//                .build();

        pathParkPose = fol.pathBuilder()
                .addPath(new BezierLine(row2Score, parkPose))
                .setLinearHeadingInterpolation(row2Score.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                if (!fol.isBusy()){
                    fol.followPath(pathPreScore);
                    setShootPos(preScorePose.getX(), preScorePose.getY(), fx, fy);
                    fol.setMaxPower(1);
                    runBelt(0);
                    setPathState(-11);
                }
                break;

            case -11:
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
                    setShootPos(row1Score.getX(), row1Score.getY(), fx, fy);
                    setPathState(1);
                }
                break;

            case 1:
                if (!fol.isBusy()) {
                    fol.followPath(pathRow1Grab);
                    fol.setMaxPower(0.32);
                    runBelt(-beltSpeed);
                    setPathState(2);
                }
                break;

            case 2:
                if (!fol.isBusy()){
                    fol.followPath(pathRow1Score);
                    fol.setMaxPower(1);
                    setPathState(21);
                }
                break;

            case 21:
                if (!fol.isBusy()){
                    setPathState(22);
                    runBelt(0);
                }
                break;

            case 22:
                if (shootTimerCount != 2)
                    shoot();
                else {
                    shootTimerCount = -1;
                    setPathState(3);
                }
                break;

            case 3:
                if (!fol.isBusy()) {
                    fol.followPath(pathRow2Line);
                    setPathState(4);
                }
                break;

            case 4:
                if (!fol.isBusy()) {
                    fol.followPath(pathRow2Grab);
                    fol.setMaxPower(0.32);
                    runBelt(-beltSpeed);
                    setPathState(5);
                }
                break;

            case 5:
                if (!fol.isBusy()){
                    fol.followPath(pathRow2Score);
                    fol.setMaxPower(1);
                    setPathState(51);
                }
                break;

            case 51:
                if (!fol.isBusy()){
                    setPathState(52);
                    runBelt(0);
                }
                break;

            case 52:
                if (shootTimerCount != 2)
                    shoot();
                else {
                    shootTimerCount = -1;
                    setPathState(9);
                }
                break;

            case 9:
                if (!fol.isBusy()) {
                    fol.followPath(pathParkPose);
                    setPathState(10);
                }
                break;

            case 10:
                if (!fol.isBusy()) {
                    setPathState(-2);
                }
                break;
        }
    }

    private void setChainNum(int num){
        chainNum = num;
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
        // Changed the multiplier to 6 instead of 8
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

    private void runBelt(double speed){
        belt.setPower(speed);
        br.setPower(speed);
        bl.setPower(-speed);
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
            runBelt(-beltSpeed);
        }
        else {
            if (ls.getVelocity() >= velToPow(shootVel) - 20 && rs.getVelocity() >= velToPow(shootVel) - 20) {
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

        telemetry.addData("X Position", "%.2f", currentX);
        telemetry.addData("Y Position", "%.2f", currentY);
    }
}