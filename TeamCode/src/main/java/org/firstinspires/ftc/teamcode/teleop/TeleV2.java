package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.FeedBackShootSystem.IDLE_VELO;
import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.closePos;
import static org.firstinspires.ftc.teamcode.subsystems.ShootSystem.openPos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FeedBackShootSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShootSystem;

import java.util.Set;




///  BLEUAFEHATGSWF why do we have so many teleop modeeasdssdsd
///
@Configurable
@TeleOp(name = "Tele V2")
public class TeleV2 extends OpMode {
    // Calls new feedbakc shoot system
    FeedBackShootSystem shooter;

    // shoot enums
    private enum ShootState {
        IDLE,
        SPIN_UP,
        FIRING
    }

    private ShootState currentShootState = ShootState.IDLE;
    private ElapsedTime shootStateTimer = new ElapsedTime();


    // Using pedros drive op for simplicity
    private Follower fol;
    private final Pose startingPose = new Pose(72, 72, Math.toRadians(0));

    // Drive init
    private DcMotorEx lb;
    private DcMotorEx rb;
    private DcMotorEx lf;
    private DcMotorEx rf;

    private final double p = 0.03, i = 0.0000, d = 0.00011;
    private double lastError;
    private double iSum;
    private boolean isShooting;

    @Override
    public void init() {
        shooter = new FeedBackShootSystem(hardwareMap, telemetry);

        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");

        // tele method i got from pedro
        fol = Constants.createFollower(hardwareMap);
        fol.setStartingPose(startingPose);
        fol.update();

        fol.startTeleOpDrive();

        shooter.beltSpeed = 0.8;
    }

    @Override
    public void loop() {
        fol.update();

        if (!isShooting) {
            fol.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
        }

        if (Math.abs(gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x) > 0.25 && isShooting) {
            fol.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
        } else if (isShooting) {
            fol.setTeleOpDrive(0, 0, 0, true);
            for (LLResultTypes.FiducialResult res : shooter.GetImage().getFiducialResults()) {
                int id = res.getFiducialId();
                if ((id == 20 || id == 24))
                    PIDAdjusting(res);
            }
        }

        if (gamepad2.a) {
            isShooting = true;
            shooter.Shoot();
        } else {
            isShooting = false;
            shooter.StopMotors();
            shooter.beltSpeed = 0.8;
            iSum = 0;
        }

        if(gamepad1.dpad_left || gamepad2.dpad_left)
            shooter.flywheel.setVelocity(-IDLE_VELO);

//        the automatic shooting mode that SHOULD work but maybe not
//        switch (currentShootState){
//            case IDLE:
//                if (gamepad1.a){
//                    shootStateTimer.reset();
//                    currentShootState = ShootState.SPIN_UP;
//                } else {
//                    shooter.StopMotors();
//                    shooter.feeder.setPosition(openPos);
//                } break;
//
//            case SPIN_UP:
//                shooter.Shoot();
//
//                // checks for if the flywheel is in an error window
//                double error = Math.abs(shooter.shootVel - shooter.flywheel.getVelocity());
//                if (error < 100){
//                    currentShootState = ShootState.FIRING;
//                    shootStateTimer.reset();
//                }
//
//                if (!gamepad1.a){
//                    currentShootState = ShootState.IDLE;
//                }
//
//                break;
//
//
//            case FIRING:
//                shooter.Shoot();
//                shooter.RunBelt(1.0);
//
//                if (shootStateTimer.milliseconds() > 700) {
//                    shooter.feeder.setPosition(FeedBackShootSystem.closePos);
//                }
//
//                if (!gamepad1.a || shootStateTimer.milliseconds() > 1500) {
//                    currentShootState = ShootState.IDLE;
//                    shooter.StopMotors();
//                    shooter.feeder.setPosition(openPos);
//                }
//                break;
//
//        }




        //belt stuff
        if (gamepad2.x)
            shooter.RunBelt(1);
        else if (gamepad2.b)
            shooter.RunBelt(-1);
        else
            shooter.RunBelt(0);

        if (gamepad2.y){
            shooter.feeder.setPosition(closePos);
        } else {
            shooter.feeder.setPosition(openPos);
        }

        // prints data
        telemetry.addData("TUNING - Servo Pos", "%.4f", shooter.manualServoPos);
        telemetry.addData("Current State", currentShootState);
        telemetry.addData("Target TPS", shooter.shootVel);
        telemetry.addData("Actual TPS", shooter.flywheel.getVelocity());
        telemetry.update();
    }

    private void PIDAdjusting(LLResultTypes.FiducialResult res){
        double error = res.getTargetXDegrees();
        iSum += error;
        double derError = lastError - error;

        lb.setPower(((error * p) + (iSum * i) + (derError * d)));
        rb.setPower(-((error * p) + (iSum * i) + (derError * d)));
        lf.setPower(((error * p) + (iSum * i) + (derError * d)));
        rf.setPower(-((error * p) + (iSum * i) + (derError * d)));

        lastError = error;
    }
}