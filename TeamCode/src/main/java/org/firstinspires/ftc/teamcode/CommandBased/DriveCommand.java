package org.firstinspires.ftc.teamcode.CommandBased;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    // define subsystem here
    private DriveSubsystem driveSubsystem;

    // Since we are getting input from joysticks we make double suppliers to get the three speeds from the controller
    private DoubleSupplier strafe, forward, turn;

    // Constructer, taking in the subsystem AND the input from the joystick doublesupplirs
    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){
        this.driveSubsystem = driveSubsystem;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;

        // says that this command uses THIS specific subsystem,
        addRequirements((driveSubsystem));
    }
    // need to overwrite execute method in command class
    @Override
    public void execute(){

        // Gets the joystick pos as double
        driveSubsystem.drive(strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());
    }
}
