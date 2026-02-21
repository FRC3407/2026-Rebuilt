// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    /** Creates a new ShooterCommand. */
    public ShooterCommand(ShooterSubsystem m_shooter) {

        shooterSubsystem = m_shooter;
        addRequirements(shooterSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterSubsystem.setShooterSpeed(1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooterSpeed(0);
    }
}
