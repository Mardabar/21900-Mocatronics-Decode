package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DcMotor;
public class Drive {

    private DcMotor lb; // R
    private DcMotor lf; // R
    private DcMotor rf;
    private DcMotor rb;
    
    private double speed;
    private double turnMult = 1.6;
    private double slowMult = 0.4;
    private double fastMult = 1.8;


    public Drive(HardwareMap hardwareMap){
        lb = hardwareMap.get(DcMotor.class, "lb");
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        
        lf = hardwareMap.get(DcMotor.class, "lf");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);

        rb = hardwareMap.get(DcMotor.class, "rb");
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        rf = hardwareMap.get(DcMotor.class, "rf");
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    
    public void update1(Gamepad gamepad){
        lb.setPower((turnMult * gamepad.right_stick_x * -speed) + (speed * gamepad.left_stick_x) + (speed * gamepad.left_stick_y));
        rb.setPower((turnMult * gamepad.right_stick_x * speed) + (-speed * gamepad.left_stick_x) + (speed * gamepad.left_stick_y));

        lf.setPower((turnMult * gamepad.right_stick_x * -speed) + (-speed * gamepad.left_stick_x) + (speed * gamepad.left_stick_y));
        rf.setPower((turnMult * gamepad.right_stick_x * speed) + (speed * gamepad.left_stick_x) + (speed * gamepad.left_stick_y));


        if (gamepad.leftStickButtonWasPressed())
            speed = slowMult;
        else if (gamepad.rightStickButtonWasPressed())
            speed = fastMult;
        else
            speed = 1;
    }
}


