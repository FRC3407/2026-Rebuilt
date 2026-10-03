// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDriveCommand extends Command {
    IntakeSubsystem intakeSubsystem;
    double speed;

    public IntakeDriveCommand(double s, IntakeSubsystem b) {
        this.speed = s;
        this.intakeSubsystem = b;
        addRequirements(getRequirements());
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeSpeed(speed);
        System.out.println("IntakeDriveCommand(" + speed + ")");

    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeSpeed(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
