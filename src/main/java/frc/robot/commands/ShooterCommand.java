// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier triggerAxis;
    /** Creates a new ShooterCommand. */
    public ShooterCommand(DoubleSupplier triggerAxis, ShooterSubsystem m_shooter) {
        this.triggerAxis = triggerAxis;
        this.shooterSubsystem = m_shooter;
        addRequirements(this.shooterSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterSubsystem.setSpindexerSpeed(MathUtil.applyDeadband(triggerAxis.getAsDouble(), OIConstants.kDriveDeadband));
        shooterSubsystem.setShooterSpeed(MathUtil.applyDeadband(triggerAxis.getAsDouble(), OIConstants.kDriveDeadband));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooterSpeed(0);
        shooterSubsystem.setSpindexerSpeed(0);
    }
}


