package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShootSubsystem;

public class FeedLauncherCommand extends CommandBase {

    private ShootSubsystem m_FeedLauncher;


    public FeedLauncherCommand(ShootSubsystem shootSystem){
        m_FeedLauncher = shootSystem;
        addRequirements();
    }







}
