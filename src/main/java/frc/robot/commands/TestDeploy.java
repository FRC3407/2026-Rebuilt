// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestDeploy extends Command {

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

  /** Creates a new TestDeploy. */
  public TestDeploy(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSubsystem = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (state == CommandState.INITIAL) {
      intakeSubsystem.setSetPoint(IntakeConstants.deployAngle);
      state = CommandState.DEPLOYING;
    }
    if (state == CommandState.DEPLOYING){
      if (deployTimer.hasElapsed(TestConstants.deployTime)){
        testFailed = true;
      }
      if (intakeSubsystem.setPointReached()) {
        state = CommandState.RETRACTING;
        intakeSubsystem.setSetPoint(0);
        deployTimer.reset();
        deployTimer.start();
      }
    }
    if (state == CommandState.RETRACTING){
      if (deployTimer.hasElapsed(TestConstants.deployTime)){
        testFailed = true;
      }
      if (intakeSubsystem.setPointReached()) {
        testPassed = true;
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (testPassed = true){
      System.out.println("Intake deploy test Passed Yippee!");
    }
    if (testFailed = true){
      System.out.println("Intake deploy test FAILED :(");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return testFailed || testPassed;
  }
}
