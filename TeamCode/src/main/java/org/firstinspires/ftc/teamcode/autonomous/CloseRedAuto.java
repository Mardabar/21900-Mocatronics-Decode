package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.paths.CloseBluePaths;
import org.firstinspires.ftc.teamcode.paths.CloseRedPaths;


@Autonomous(name = "CloseRed")
public class CloseRedAuto extends OpMode{


    // PEDRO VARS
    private CloseRedPaths paths;
    private Follower fol;
    private int pathState; // Current path #


    // MOTORS

    DcMotorEx belt, cannon;

    // SERVOS

    Servo blocker;


    @Override
    public void init(){



        // Initialize new motors and servos here
        cannon = hardwareMap.get(DcMotorEx.class, "cannon");
        MotorConfigurationType configRs = cannon.getMotorType().clone();
        configRs.setAchieveableMaxRPMFraction(1.0);
        cannon.setMotorType(configRs);
        cannon.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cannon.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        belt = hardwareMap.get(DcMotorEx.class, "belt");


        blocker = hardwareMap.get(Servo.class, "blocker");





        // Pedro paths init
        paths = new CloseRedPaths(fol);
        setPathState(0);

    }


    public void loop(){


        // Pedro runs paths
        autonomousPathUpdate();
    }



    // Negative path state just means its in done with the scoring check phase and is currently shooting
    private void autonomousPathUpdate(){
        switch (pathState){
            // Bot moves from starting to prescore
            case 0:
                if (!fol.isBusy()) {
                    fol.followPath(paths.pathPreScore);
                    setPathState(1);
                } break;

            // Bot will do a check if its not moving here
            case 1:
                if (!fol.isBusy()){
                    setPathState(-1);
                } break;

            // Bot will score here
            case -1:
                ///  ADD SCORING CODE FUNC ITS MADE AND TESTED
                // Shoot function goes here with timer to make sure it will shoot for the time it needs
                if (!fol.isBusy()){
                    setPathState(2);
                } break;

            // Bot lines to row 1
            case 2:
                if (!fol.isBusy()){
                    fol.followPath(paths.pathRow1Line);
                    setPathState(3);
                } break;

            // Bot moves and grabs row 1
            case 3:
                if (!fol.isBusy()){
                    ///  ADD INTAKE RUNNING HERE
                    fol.followPath(paths.pathRow1Grab);
                    setPathState(4);
                } break;

            // Bot goes to score pos
            case 4:
                if(!fol.isBusy()){
                    fol.followPath(paths.pathRow1Score);
                    setPathState(-5);
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
                    fol.followPath(paths.pathRow2Line);
                    setPathState(7);
                } break;

            // Bot grabs the balls in row 2
            case 7:
                if(!fol.isBusy()){
                    fol.followPath(paths.pathRow2Grab);
                    /// Intake system run here
                    runBelt(.8);
                    setPathState(8);
                } break;

            // Bot goes to scoring pos
            case 8:
                if(!fol.isBusy() && pathState == 8){
                    stopBelt();
                    fol.followPath(paths.pathRow2Score);
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
                    fol.followPath(paths.pathRow3Line);
                    setPathState(10);
                } break;

            // Bot grabs the balls in row 3
            case 10:
                if(!fol.isBusy()){
                    fol.followPath(paths.pathRow3Grab);
                    /// Intake system run here
                    runBelt(.8);
                    setPathState(11);
                } break;

            // Bot goes to scoring pos
            case 11:
                if(!fol.isBusy() && pathState == 11){
                    stopBelt();
                    fol.followPath(paths.pathRow3Score);
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
                    fol.followPath(paths.pathPark);
                    setPathState(14);
                } break;


        }

    }


    /// PEDRO FUNCTIONS
    private void setPathState(int num){
        pathState = num;
    }


    /// INTAKE FUNCTIONS
    private void runBelt(double pow){
        belt.setPower(pow);
    }

    private void stopBelt(){
        belt.setPower(0);
    }

    /// BLOCKER SERVO FUNCTIONS
    // Will have to find specific positions
    private void openGate(){
        blocker.setPosition(.04); // Mess w this later
    }

    private void closeGate(){
        blocker.setPosition(.03); // Mess w these
    }

}

