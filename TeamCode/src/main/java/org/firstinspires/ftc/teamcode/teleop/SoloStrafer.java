package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShootSystem;
import org.firstinspires.ftc.teamcode.subsystems.SoloShoot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Configurable
@TeleOp(name = "SoloStrafer")
public class SoloStrafer extends OpMode {

    // Subsystem stuff
    private Shooter shooter;
    private Intake intake;
    private Drive drive;


    private SoloShoot soloShoot;
    private ShootSystem shoot;

    private DcMotor belt;
    private CRServo br;
    private CRServo bl;
    private TelemetryManager telemetryM;




    @Override
    public void init() {

        // Subsystem stuff
        //soloShoot = new SoloShoot(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        drive = new Drive(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


//        belt = hardwareMap.get(DcMotor.class, "belt");
//        br = hardwareMap.get(CRServo.class, "br");
//        bl = hardwareMap.get(CRServo.class, "bl");


    }

        @Override
        public void loop(){




            //soloShoot.update1(gamepad1);
            shooter.update(gamepad1);
            intake.update1(gamepad1);
            drive.update1(gamepad1);
            updateTele();

//
//            if (gamepad1.x)
//                runBelt(1);
//            else if (gamepad1.b)
//                runBelt(-1);
//            else
//                runBelt(0);
        }



    private void runBelt(double speed) {
        belt.setPower(speed);
        br.setPower(speed);
        bl.setPower(-speed);
    }


    private void updateTele(){
        shooter.updateTelemetry(telemetryM);
        telemetryM.update();
    }
}

