package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.closePos;
import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.openPos;
import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.shootVel;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

//import org.firstinspires.ftc.teamcode.paths.CloseBluePaths;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShootSystem;


@Autonomous(name = "CloseBlue")
public class CloseBlueAuto extends OpMode{

    /// PATHS
    private final Pose startPose = new Pose(27.3, 132.7, Math.toRadians(143));
    private final Pose preScorePose = new Pose(50, 115, Math.toRadians(143));
    private final Pose row1Line = new Pose(48, 84, Math.toRadians(180));
    private final Pose row1Grab = new Pose(21, 84, Math.toRadians(180));
    private final Pose row1Score = new Pose(39.5, 102, Math.toRadians(135));
    private final Pose row2Line = new Pose(48, 59.5, Math.toRadians(180));
    private final Pose row2Grab = new Pose(21, 59.5, Math.toRadians(180));
    private final Pose row2Score = new Pose(52, 88.5, Math.toRadians(135));
    private final Pose row3Line = new Pose (48, 35.5, Math.toRadians(180));
    private final Pose row3Grab = new Pose (20, 35.5, Math.toRadians(180));

    /// Row 3 score and park close
    private final Pose row3ScoreClose = new Pose (57.5, 84.3, Math.toRadians(134));
    private final Pose row3ParkClose = new Pose (55, 63, Math.toRadians(134));

    /// Row 3 score and park far
    private final Pose row3ScoreFar = new Pose (58, 13.5, Math.toRadians(124));
    private final Pose row3ParkFar = new Pose (55.5, 39, Math.toRadians(124));

    // PEDRO VARS
    //private CloseBluePaths paths;
    private Follower fol;
    private int pathState; // Current path #
    private PathChain pathPreScore, pathRow1Line, pathRow1Grab, pathRow1Score, pathRow2Line, pathRow2Grab, pathRow2Score, pathRow3Line, pathRow3Grab,  pathRow3Score, pathPark;



    // MOTORS

    DcMotorEx belt, cannon;


    // SERVOS

    Servo feeder;
    Servo angleAdjuster;

    private ShootSystem shooter;


    private double shootDur = 5000;
    private double flipperDelay = 3000;



    // Time
    private ElapsedTime shootTimer, blockTimer;

    private double blockDur = 2000;

    private int shootCount = -1;


    @Override
    public void init(){


        // Fol init
        //paths = new CloseBluePaths(fol);

        shooter = new ShootSystem(hardwareMap, telemetry);

        fol = Constants.createFollower(hardwareMap);
        fol.setStartingPose(startPose);



        // Initialize new motors and servos here
        cannon = hardwareMap.get(DcMotorEx.class, "cannon");
        MotorConfigurationType configRs = cannon.getMotorType().clone();
        configRs.setAchieveableMaxRPMFraction(1.0);
        cannon.setMotorType(configRs);
        cannon.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cannon.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        belt = hardwareMap.get(DcMotorEx.class, "belt");
        belt.setDirection(DcMotorSimple.Direction.REVERSE);


        feeder = hardwareMap.get(Servo.class, "feeder");




        shootTimer = new ElapsedTime();
        blockTimer = new ElapsedTime();

        // Pedro paths init
        buildPaths();
        setPathState(0);

    }


    public void loop(){


        // Pedro runs paths
        fol.update();
        autonomousPathUpdate();
    }



    // Negative path state just means its in done with the scoring check phase and is currently shooting
    private void autonomousPathUpdate(){
        switch (pathState){
            // Bot moves from starting to prescore
            case 0:
                if (!fol.isBusy()) {
                    fol.followPath(pathPreScore);
                    setPathState(1);
                } break;

            // Bot will do a check if its not moving here
            case 1:
                if (!fol.isBusy()){
                    setPathState(-1);
                } break;

            // Bot will score here
            case -1:
                shootBalls();
                if (shootCount == 2) {
                    shootCount = -1;
                    setPathState(2);
                }
                break;

                /*
                *             case -2:
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
                * */
            // Bot lines to row 1
            case 2:
                if (!fol.isBusy()){
                    fol.followPath(pathRow1Line);
                    setPathState(3);
                } break;

            // Bot moves and grabs row 1
            case 3:
                if (!fol.isBusy()){
                    ///  ADD INTAKE RUNNING HERE
                    fol.setMaxPower(.4);
                    runbelt(1);
                    fol.followPath(pathRow1Grab);
                    setPathState(4);
                } break;

            // Bot goes to score pos
            case 4:
                if(!fol.isBusy()){
                    fol.followPath(pathRow1Score);
                    stopBelt();
                    setPathState(-52);
                } break;

            // Bot checks if its not moving
            case -5:
                if(!fol.isBusy()){
                    setPathState(5);
                } break;

            // Bot does shooting here, need to add timer to check when the bot can move again
            case 5:
                if(!fol.isBusy()){
                    // Shoot function goes here with timer to make sure it will shoot for the time it needs
                    setPathState(6);
                } break;

            // Bot will go to line with row 2
            case 6:
                if(!fol.isBusy() && pathState == 6){
                    fol.followPath(pathRow2Line);
                    setPathState(7);
                } break;

            // Bot grabs the balls in row 2
            case 7:
                if(!fol.isBusy()){
                    fol.followPath(pathRow2Grab);
                    /// Intake system run here
                    runbelt(1);
                    setPathState(8);
                } break;

            // Bot goes to scoring pos
            case 8:
                if(!fol.isBusy() && pathState == 8){
                    stopBelt();
                    fol.followPath(pathRow2Score);
                    setPathState(-8);
                } break;

                // Bot does shooting here, need to add timer to check when the bot can move again
            case -8:
                if(!fol.isBusy()){
                    // Shoot function goes here with timer to make sure it will shoot for the time it needs
                    setPathState(9);
                } break;

            // Bot will go to line with row 3
            case 9:
                if(!fol.isBusy() && pathState == 9){
                    fol.followPath(pathRow3Line);
                    setPathState(10);
                } break;

            // Bot grabs the balls in row 3
            case 10:
                if(!fol.isBusy()){
                    fol.followPath(pathRow3Grab);
                    /// Intake system run here
                    runbelt(0.8);
                    setPathState(11);
                } break;

            // Bot goes to scoring pos
            case 11:
                if(!fol.isBusy() && pathState == 11){
                    stopBelt();
                    fol.followPath(pathRow3Score);
                    setPathState(-11);
                } break;

            // Bot checks for it to stop moving
            case -11:
                if(!fol.isBusy()){
                    setPathState(12);
                } break;

                // Bot does shooting here, need to add timer to check when the bot can move again
            case 12:
                if(!fol.isBusy()){
                    // Shoot function goes here with timer to make sure it will shoot for the time it needs
                    setPathState(13);
                } break;

            // Bot goes to park
            case 13:
                if(!fol.isBusy()){
                    fol.followPath(pathPark);
                    setPathState(14);
                } break;


        }

    }


    /// PEDRO FUNCTIONS
    private void setPathState(int num){
        pathState = num;
    }


    /// INTAKE FUNCTIONS
    private void runbelt(double pow){
        belt.setPower(pow);
    }

    private void stopBelt(){
        belt.setPower(0);
    }

    /// feeder SERVO FUNCTIONS
    // Will have to find specific positions




    private void buildPaths(){
        pathPreScore = fol.pathBuilder()
                .addPath(new BezierLine(startPose, preScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
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

        pathRow3Line = fol.pathBuilder()
                .addPath(new BezierLine(row2Score, row3Line))
                .setLinearHeadingInterpolation(row2Score.getHeading(), row3Line.getHeading())
                .build();

        pathRow3Grab = fol.pathBuilder()
                .addPath(new BezierLine(row3Line, row3Grab))
                .setLinearHeadingInterpolation(row3Line.getHeading(), row3Grab.getHeading())
                .build();

        //              Row 3 close score and shoot
        pathRow3Score = fol.pathBuilder()
                .addPath(new BezierLine(row3Grab, row3ScoreClose))
                .setLinearHeadingInterpolation(row3Grab.getHeading(), row3ScoreClose.getHeading())
                .build();

        pathPark = fol.pathBuilder()
                .addPath(new BezierLine(row3ScoreClose, row3ParkClose))
                .setLinearHeadingInterpolation(row3ScoreClose.getHeading(), row3ParkClose.getHeading())
                .build();

        /*              Row 3 far score and shoot
        pathRow3Score = fol.pathBuilder()
                .addPath(new BezierLine(row3Grab, row3ScoreFar))
                .setLinearHeadingInterpolation(row3Grab.getHeading(), row3ScoreFar.getHeading())
                .build();

        pathPark = fol.pathBuilder()
                .addPath(new BezierLine(row3ScoreFar, row3ParkFar))
                .setLinearHeadingInterpolation(row3ScoreFar.getHeading(), row3ParkFar.getHeading())
                .build();  */



    }


    private void shootBalls() {
        if (shootCount == -1) {
            shootTimer.reset();
            blockTimer.reset();

            shootCount = 0;
        }

        double currentTime = shootTimer.milliseconds();

        if (currentTime < shootDur) {
            shooter.Shoot();

            if (blockTimer.milliseconds() < flipperDelay) {
                feeder.setPosition(openPos);
            }

        } else {
            // Stop everything after shootDur
            shooter.StopMotors();
            feeder.setPosition(closePos);
            shootCount = 2;
        }
    }
}

