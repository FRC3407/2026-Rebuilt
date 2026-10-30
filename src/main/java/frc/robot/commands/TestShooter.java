// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.TestConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestShooter extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final Timer timer;
    private Boolean testFailed = null;

    /** Creates a new TestShooter. */
    public TestShooter(ShooterSubsystem shooter) {
        this.shooterSubsystem = shooter;
        timer = new Timer();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        testFailed = null;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double shooterSpeed = shooterSubsystem.getShooterSpeed();
        if (timer.hasElapsed(6)) {
            shooterSubsystem.setShooterSpeed(-1);
            if (shooterSpeed > -TestConstants.shooterSpeedThreshold) {
                testFailed = true;
            }
        } else if (timer.hasElapsed(4)) {
            shooterSubsystem.setShooterSpeed(-1);

        } else if (timer.hasElapsed(2)) {
            shooterSubsystem.setShooterSpeed(1);
            if (shooterSpeed < TestConstants.shooterSpeedThreshold) {
                testFailed = true;
            }
        } else {
            shooterSubsystem.setShooterSpeed(1);

        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("testShooter = " + testFailed);
        shooterSubsystem.setShooterSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (testFailed) {
            return true;
        }
        if (timer.hasElapsed(8)) {
            testFailed = false;
            return true;
        }
        return false;
    }

    private String getTestStatus() {
        if(testFailed) {
            return "failed";
        } else if (!timer.hasElapsed(7)) {
        return "test running";
        } else { return "pass"; }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("shooter test status", this::getTestStatus, null);
    }
}
