package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MotorTest")

public class MotorTest extends OpMode {

    private DcMotorEx belt, cannon;
    private double launchPow = .5;


    @Override
    public void init(){

        belt = hardwareMap.get(DcMotorEx.class, "belt");
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        belt.setDirection(DcMotorEx.Direction.REVERSE);

        cannon = hardwareMap.get(DcMotorEx.class, "cannon");
        cannon.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cannon.setDirection(DcMotorEx.Direction.REVERSE);



    }

    @Override
    public void loop(){

        updateTele();

        if(gamepad1.x)
            runBelt(1);
        else if(gamepad1.b)
            runBelt(-1);
        else
            stopBelt();

        if (gamepad1.dpadUpWasPressed())
            increaseLaunchPow();
        else if(gamepad1.dpadDownWasPressed())
            decreaseLaunchPow();


        if(gamepad1.aWasPressed())
            cannon.setPower(launchPow);

        if (gamepad1.yWasPressed())
            cannon.setPower(0);


    }


    private void runBelt(double pow){
        belt.setPower(pow);
    }

    private void stopBelt(){
        belt.setPower(0);
    }

    private void increaseLaunchPow(){
        launchPow+= .1;
    }
    private void decreaseLaunchPow(){
        launchPow-= .1;
    }

    private void updateTele(){
        telemetry.addData("Launch power: ", launchPow);
        telemetry.update();
    }

}
