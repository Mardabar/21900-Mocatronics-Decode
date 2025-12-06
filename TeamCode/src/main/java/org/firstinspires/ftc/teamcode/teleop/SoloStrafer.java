package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.subsystems.Drive;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SoloShoot;


@Configurable
@TeleOp(name = "SoloStrafer")
public class SoloStrafer extends LinearOpMode {

    // Subsystem stuff
    private Intake intake;
    private SoloShoot soloShoot;
    private Drive drive;


    @Override
    public void runOpMode() {

        // Subsystem stuff
        soloShoot = new SoloShoot(hardwareMap);
        intake = new Intake(hardwareMap);
        drive = new Drive(hardwareMap);



        // Robot waits for the opmode to be activated
        waitForStart();

        while (opModeIsActive()) {
            soloShoot.update1(gamepad1);
            intake.update1(gamepad1);
            drive.update1(gamepad1);
        }
    }
}
