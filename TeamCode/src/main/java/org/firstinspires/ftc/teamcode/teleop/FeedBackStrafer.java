package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.closePos;
import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.openPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FeedBackShootSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShootSystem;


///  BLEUAFEHATGSWF why do we have so many op modeeasdssdsd
///
@Configurable
@TeleOp(name = "FeedBack Strafer")
public class FeedBackStrafer extends OpMode {


    // Calls new feedbakc shoot system thing
    FeedBackShootSystem shooter;
    private Servo angleAdjuster;



    private Follower fol;
    private final Pose startingPose = new Pose(72, 72, Math.toRadians(0));
    private double mainSpeed = 1;


    @Override
    public void init() {
        shooter = new FeedBackShootSystem(hardwareMap, telemetry);

        fol = Constants.createFollower(hardwareMap);
        fol.setStartingPose(startingPose == null ? new Pose() : startingPose);
        fol.update();






        fol.startTeleOpDrive();


        // tele method i got from docs


    }

    @Override
    public void loop() {


        fol.update();
        fol.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );

        // the automatic shooting mode that SHOULD work but maybe not
        if (gamepad1.a) {
            shooter.Shoot();
        } else {
            shooter.StopMotors();
        }




        //belt stuff
        if (gamepad1.x)
            shooter.RunBelt(1);
        else if (gamepad1.b)
            shooter.RunBelt(-1);
        else
            shooter.RunBelt(0);

        if (gamepad1.y){
            shooter.feeder.setPosition(closePos);
        } else {
            shooter.feeder.setPosition(openPos);
        }

//        shooter.moveAngleManual(gamepad2.left_stick_y);




        // prints data
        telemetry.addData("Target TPS", ShootSystem.shootVel);
        telemetry.addData("Actual TPS", shooter.flywheel.getVelocity());

        //telemetry.addData("Bot Pose in space", shooter.cam.getLatestResult());
        telemetry.update();



        // errrrrr drive code i have to throw tg very quick

    }




}