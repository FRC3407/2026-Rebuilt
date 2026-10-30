// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.IntakeSubsystem;

/** Tests whether the intake deploys or not */
public class TestIntakeDeploy extends Command {

    private IntakeSubsystem intakeSubsystem;
    private boolean testFailed = false;
    private boolean testPassed = false;
    private Timer TestTimer = new Timer();
    private Timer deployTimer = new Timer();

    enum CommandState {
        INITIAL,
        RETRACTING,
        DEPLOYING

    }

    CommandState state = CommandState.INITIAL;

    public TestIntakeDeploy(IntakeSubsystem intake) {
        intakeSubsystem = intake;
        addRequirements(intake);
    }
    @Override
    public void initialize() {
        testFailed = false;
        testPassed = false;
        deployTimer.reset();
        deployTimer.start();
        TestTimer.reset();
        TestTimer.start();
        state = CommandState.INITIAL;
    }
    @Override
    public void execute() {
        if (state == CommandState.INITIAL) {
            intakeSubsystem.setSetPoint(IntakeConstants.deployAngle);
            state = CommandState.DEPLOYING;
        }
        if (state == CommandState.DEPLOYING) {
            if (deployTimer.hasElapsed(TestConstants.deployTime)) {
                testFailed = true;
            }
            if (intakeSubsystem.setPointReached()) {
                state = CommandState.RETRACTING;
                intakeSubsystem.setSetPoint(0);
                deployTimer.reset();
                deployTimer.start();
            }
        }
        if (state == CommandState.RETRACTING) {
            if (deployTimer.hasElapsed(TestConstants.deployTime)) {
                testFailed = true;
            }
            if (intakeSubsystem.setPointReached()) {
                testPassed = true;
            }
        }

    }
    @Override
    public void end(boolean interrupted) {
        if (testPassed = true) {
            System.out.println("Intake deploy test Passed Yippee!:3");
        }
        if (testFailed = true) {
            System.out.println("Intake deploy test FAILED :(");
        }
    }
    @Override
    public boolean isFinished() {
        return testFailed || testPassed;
    }
}
