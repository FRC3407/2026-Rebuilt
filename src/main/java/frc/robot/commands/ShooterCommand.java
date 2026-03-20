// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier triggerAxis;

    private final Timer time = new Timer();

    /** Creates a new ShooterCommand. */
    public ShooterCommand(DoubleSupplier triggerAxis, ShooterSubsystem m_shooter) {
        this.triggerAxis = triggerAxis;
        this.shooterSubsystem = m_shooter;
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        time.reset();
        time.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final double howMuchTrigger = MathUtil.applyDeadband(triggerAxis.getAsDouble(), OIConstants.kDriveDeadband);
        shooterSubsystem.setShooterSpeed(howMuchTrigger);

        // delay for spindexer of 0.5 seconds (hope that my ternary operator isn't too scawy)
        shooterSubsystem.setSpindexerSpeed(time.hasElapsed(.5) ? 1 : 0);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooterSpeed(0);
        shooterSubsystem.setSpindexerSpeed(0);
    }
}
