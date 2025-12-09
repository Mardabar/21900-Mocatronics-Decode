package org.firstinspires.ftc.teamcode.CommandBased;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveSubsystem extends SubsystemBase {
    private MecanumDrive drive;
    private Motor lb, lf, rf, rb;




    // Init the drive system
    public DriveSubsystem(Motor Leftb, Motor Leftf,Motor Rightf, Motor Rightb){
        lb = Leftb;
        lf = Leftf;
        rf = Rightf;
        rb = Rightb;

        drive = new MecanumDrive(lb, lf, rf, rb);


    }
    // Here we set params for the speeds of the drive func
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed){
        // robotcentric is easier and less work/easier but can be changed
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }
}
