package org.firstinspires.ftc.teamcode.subsystems;



import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@Configurable
public class Intake {

    private DcMotor belt;
    private CRServo br;
    private CRServo bl;

    public static double speed;

    // Initializng motor and servos here
    public Intake(HardwareMap hardwareMap){
        // Belt motor init
        belt = hardwareMap.get(DcMotor.class, "belt");
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        belt.setDirection(DcMotor.Direction.FORWARD);


        // Servo init
        br = hardwareMap.get(CRServo.class, "br");
        bl = hardwareMap.get(CRServo.class, "br");

    }


    public void update1(Gamepad gamepad){
        if(gamepad.b){
            belt.setPower(1);
            br.setPower(1);
            bl.setPower(-1);
        } else if (gamepad.x){
            belt.setPower(-1);
            br.setPower(-1);
            bl.setPower(1);
        } else {
            belt.setPower(0);
            br.setPower(0);
            bl.setPower(0);
        }
    }

    private void runBelt(double speed) {
        belt.setPower(speed);
        br.setPower(speed);
        bl.setPower(-speed);
    }




}