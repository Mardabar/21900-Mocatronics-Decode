package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.closePos;
import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.openPos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FeedBackShootSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShootSystem;


///  BLEUAFEHATGSWF why do we have so many teleop modeeasdssdsd
///
@Configurable
@TeleOp(name = "Tele V2")
public class TeleV2 extends OpMode {


    // Calls new feedbakc shoot system
    FeedBackShootSystem shooter;




    // Using pedros drive op for simplicity
    private Follower fol;
    private final Pose startingPose = new Pose(72, 72, Math.toRadians(0));



    @Override
    public void init() {
        shooter = new FeedBackShootSystem(hardwareMap, telemetry);

        // tele method i got from pedro
        fol = Constants.createFollower(hardwareMap);
        fol.setStartingPose(startingPose);
        fol.update();

        fol.startTeleOpDrive();




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




        // prints data
        telemetry.addData("Target TPS", ShootSystem.shootVel);
        telemetry.addData("Actual TPS", shooter.flywheel.getVelocity());
        //telemetry.addData("Bot Pose in space", shooter.cam.getLatestResult());
        telemetry.update();

    }
}