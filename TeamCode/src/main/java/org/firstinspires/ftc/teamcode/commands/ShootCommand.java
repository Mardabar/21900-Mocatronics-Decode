package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShootSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShootSystem;

import dev.nextftc.core.commands.Command;

public class ShootCommand extends CommandBase {

    private ShootSubsystem m_shoot;

    public ShootCommand(ShootSubsystem shootSystem){
        m_shoot = shootSystem;
        addRequirements();
    }

    @Override
    public void execute(){
        /// FAHGHHHHHHHHHHHHHHHHHHHHHHHH it works
        m_shoot.feedLauncher();
    }
}
