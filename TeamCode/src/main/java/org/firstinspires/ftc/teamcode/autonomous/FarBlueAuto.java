package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FeedBackShootSystem;


@Autonomous(name = "FarBlue")
public class FarBlueAuto extends OpMode {

    ///  Paths

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose preScorePose = new Pose(63, 18, Math.toRadians(114)); // for close shooting do 62, 119, 159
    private final Pose row3Line = new Pose (48, 36, Math.toRadians(180));
    private final Pose row3Grab = new Pose (11, 36, Math.toRadians(180));
    private final Pose row3Score = new Pose(63, 18, Math.toRadians(114)); // 55.5, 121. 159
    private final Pose row1Line = new Pose(48, 84, Math.toRadians(180));
    private final Pose row1Grab = new Pose(18.5, 84, Math.toRadians(180));
    private final Pose row1Score = new Pose(39.5, 102, Math.toRadians(135));
    private final Pose row2Line = new Pose(48, 60, Math.toRadians(180));
    private final Pose row2Grab = new Pose(13, 60, Math.toRadians(180));
    private final Pose row2Score = new Pose(50, 93, Math.toRadians(135)); // was 52, 88.5, 135
    private final Pose row2ScoreCP = new Pose(53, 58);

    /// Row 3 score and park close
    private final Pose row3ScoreClose = new Pose (48, 107, Math.toRadians(138));
    private final Pose row3ScoreCP = new Pose(102, 70);



    // Pedro vars
    private PathChain pathPreScore, pathRow1Line, pathRow1Grab, pathRow1Score, pathRow2Line, pathRow2Grab, pathRow2Score, pathRow3Line, pathRow3Grab,  pathRow3Score, pathPark;
    private Follower fol;
    private int pathState; // Current path #

    // Subsystem
    private FeedBackShootSystem shooter;

    // Timeers
    private ElapsedTime shootTimer, beltTimer;


    @Override
    public void init(){


        shooter = new FeedBackShootSystem(hardwareMap, telemetry);

        fol = Constants.createFollower(hardwareMap);
        fol.setStartingPose(startPose);
        shootTimer = new ElapsedTime();
        beltTimer = new ElapsedTime();

        // Pedro paths init
        buildPaths();
        setPathState(0);

    }




    @Override
    public void loop(){

        fol.update();
        autonomousPathUpdate();

    }



    private void autonomousPathUpdate(){
        switch(pathState){


            case 0:
                if (!fol.isBusy()) {
                    fol.followPath(pathPreScore);
                    setPathState(1);
                } break;

            case 1:
                if (!fol.isBusy()){
                    shootTimer.reset();
                    setPathState(-1);
                } break;

            case -1:
                shootFar(2); // change back to 2
                break;

            case 2:
                if (!fol.isBusy()){
                    fol.followPath(pathRow3Line);
                    setPathState(3);
                } break;

            case 3:
                if (!fol.isBusy()){
                    fol.setMaxPower(.6);
                    shooter.RunBelt(.35);
                    fol.followPath(pathRow3Grab);
                    beltTimer.reset();
                    setPathState(4); // back to 4
                } break;

            // Bot goes to scoring pos
            case 4:
                if(!fol.isBusy()){
                    fol.setMaxPower(1);
                    shooter.stopBelt();
                    //stopBelt();
                    fol.followPath(pathRow3Score);
                    setPathState(-4);
                } break;

            // Bot checks for it to stop moving
            case -4:
                if(!fol.isBusy()){
                    shootTimer.reset();
                    setPathState(5);
                } break;

            // Bot does shooting here, need to add timer to check when the bot can move again
            case 5:
                if(!fol.isBusy()){
                    shootFar(6);
                } break;

            case 6:
                if (!fol.isBusy()){
                    fol.followPath(pathRow2Line);
                    setPathState(7);
                } break;

            case 7:
                if (!fol.isBusy()){
                    fol.setMaxPower(.6);
                    shooter.RunBelt(.35);
                    fol.followPath(pathRow2Grab);
                    beltTimer.reset();
                    setPathState(8); // back to 4
                } break;

            // Bot goes to scoring pos
            case 8:
                if(!fol.isBusy()){
                    fol.setMaxPower(1);
                    shooter.stopBelt();
                    //stopBelt();
                    fol.followPath(pathRow2Score);
                    setPathState(-8);
                } break;

            // Bot checks for it to stop moving
            case -8:
                if(!fol.isBusy()){
                    shootTimer.reset();
                    setPathState(9);
                } break;

            // Bot does shooting here, need to add timer to check when the bot can move again
            case 9:
                if(!fol.isBusy()){
                    shootFar(10);
                } break;


        }


    }


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
                .addPath(new BezierCurve(row2Grab, row2ScoreCP, row2Score))
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


    private void shoot(int nextState) {
        // updates and sets motors to power
        shooter.Shoot();

        if (shootTimer.milliseconds() > 900) {
            shooter.feeder.setPosition(FeedBackShootSystem.closePos);
        } else {
            shooter.feeder.setPosition(FeedBackShootSystem.openPos);
        }

        // lets the flywheel spin up for a bit might need to make bigger
        if (shootTimer.milliseconds() < 500) {
            shooter.stopBelt();
        }

        // after that checks if the flywheel is at the velocity or if we have spun for over 3 seconds
        else if (Math.abs(shooter.shootVel - shooter.flywheel.getVelocity()) < 500 || shootTimer.milliseconds() > 700) {
            shooter.RunBelt(0.8);

        }

        // After 4 seconds stop everything and move to the next path state incase sum gets messed up
        if (shootTimer.milliseconds() > 1900) {
            shooter.StopMotors();
            shooter.feeder.setPosition(FeedBackShootSystem.openPos);
            setPathState(nextState);
        }

    }


    private void shootFar(int nextState) {
        // updates and sets motors to power
        shooter.Shoot();

        if (shootTimer.milliseconds() > 3000) {
            shooter.feeder.setPosition(FeedBackShootSystem.closePos);
        } else {
            shooter.feeder.setPosition(FeedBackShootSystem.openPos);
        }

        // lets the flywheel spin up for a bit might need to make bigger
        if (shootTimer.milliseconds() < 500) {
            shooter.stopBelt();
        }

        // after that checks if the flywheel is at the velocity or if we have spun for over 3 seconds
        else if (Math.abs(shooter.shootVel - shooter.flywheel.getVelocity()) < 50 || shootTimer.milliseconds() > 1200) {
            shooter.RunBelt(0.35);

        }

        // After 4 seconds stop everything and move to the next path state incase sum gets messed up
        if (shootTimer.milliseconds() > 3500) {
            shooter.StopMotors();
            shooter.feeder.setPosition(FeedBackShootSystem.openPos);
            setPathState(nextState);
        }
    }

    /// PEDRO FUNCTIONS
    private void setPathState(int num){
        pathState = num;
    }


}
