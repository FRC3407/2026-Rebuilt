// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterDataConstants;

public class AutoShootCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final DriveSubsystem driveSubsystem;

    private final Timer time = new Timer();
    
    /** Creates a new AutoShootCommand. */
    public AutoShootCommand(ShooterSubsystem m_shooter, DriveSubsystem m_drive) {
        shooterSubsystem = m_shooter;
        driveSubsystem = m_drive;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        time.reset();
        time.start();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final double targetVel = getTargetVelocity();
        final double targetShooterSpeed = velocityToPower(targetVel);
        shooterSubsystem.setShooterSpeed(targetShooterSpeed);

        // delay for spindexer of 0.5 seconds (hope that my ternary operator isn't too scawy)
        shooterSubsystem.setSpindexerSpeed(time.hasElapsed(.75) ? 1 : 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooterSpeed(0);
        shooterSubsystem.setSpindexerSpeed(0);
    }

    // PHYSICS!!! :ь
    private double getTargetVelocity() {
        final double distanceFrom = driveSubsystem.distanceToHub();
        final double earthGravity = 9.80665; // m/s^2
        
                                        // this is a super guess-y value
        final double d = distanceFrom - Units.inchesToMeters(20) + ShooterConstants.hubDiameter / 2; // COULD BE either the outer or inner limit, decide/make other case later
        final double hToHub = ShooterConstants.hubHeight - ShooterConstants.launcherHeight;
        final double theta = ShooterConstants.launchAngle;

        final double outside = d / Math.cos(theta);
        final double tangentDenominator = d * Math.tan(theta) - hToHub;
        final double squareroooooooooot = Math.sqrt(earthGravity / tangentDenominator); // yay... im so excited... big calv made my code so good...

        final double solvedVelocity = outside * squareroooooooooot;
        return solvedVelocity;
    }

    private double velocityToPower(double desiredVelocity) {
        return ShooterDataConstants.trapezoidApproximation(ShooterDataConstants.rpmToVelocity, desiredVelocity);
    }
}
