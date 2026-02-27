// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final DriveSubsystem driveSubsystem;
    /** Creates a new ShooterCommand. */
    public ShooterCommand(ShooterSubsystem m_shooter,DriveSubsystem m_drive) {

        shooterSubsystem = m_shooter;
        driveSubsystem = m_drive;
        addRequirements(shooterSubsystem);
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
