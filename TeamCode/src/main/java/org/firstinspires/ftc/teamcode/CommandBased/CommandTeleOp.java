package org.firstinspires.ftc.teamcode.CommandBased;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="CommandTeleOp")
public class CommandTeleOp extends CommandOpMode {
    //Calling motors, subsystems and commands
    private Motor  lb, lf, rf, rb;
    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    // Now need controller feedback
    private GamepadEx m_driverOp;

    @Override
    public void initialize(){

        lb = new Motor(hardwareMap, "lb");
        lf = new Motor(hardwareMap, "lf");
        rb = new Motor(hardwareMap, "rb");
        rf = new Motor(hardwareMap, "lf");


        lb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        m_driverOp = new GamepadEx(gamepad1);

        driveSubsystem = new DriveSubsystem(lb, lf, rb, rf);
        driveCommand = new DriveCommand(driveSubsystem, m_driverOp::getLeftX, m_driverOp::getLeftY, m_driverOp::getRightX);


        // setting a default command so itll always run this, and registering it just in case
        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
    }

}
