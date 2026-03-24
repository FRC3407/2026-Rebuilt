// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AnotherAutoShootCommand extends Command {
  /** Creates a new ShootTestCommand. */

  private final ShooterSubsystem shooterSubsystem;
  private final DriveSubsystem driveSubsystem;
  private Timer time = new Timer();

  public static double getShooterSpeed(double distance) {
    return 0.12541 * distance + 0.428443;
  }

  public AnotherAutoShootCommand(ShooterSubsystem m_shooter, DriveSubsystem m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = m_shooter;
    this.driveSubsystem = m_drive;
    SmartDashboard.putData("Auto Shoot Command", this);
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("auto shoot speed", () -> getShooterSpeed(driveSubsystem.distanceToHub()), null);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setShooterSpeed(getShooterSpeed(driveSubsystem.distanceToHub()));

    if (time.hasElapsed(1.5)) {
      shooterSubsystem.setSpindexerSpeed(1);
    } else {
      shooterSubsystem.setSpindexerSpeed(0);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterSpeed(0);
    shooterSubsystem.setSpindexerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
