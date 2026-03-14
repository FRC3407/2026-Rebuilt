// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeployCommand extends InstantCommand {
    private IntakeSubsystem intakeSubsystem;

    private double setPoint = 0;

    public DeployCommand(IntakeSubsystem intake, double setPoint) {
        intakeSubsystem = intake;
        addRequirements(intake);
        this.setPoint = setPoint;
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intakeSubsystem.setSetPoint(setPoint);
    }
}
