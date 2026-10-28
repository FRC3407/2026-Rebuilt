// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionTestCommand extends Command {
  private TestProgress progress;
  private String message;
private final VisionSubsystem visionSubsystem; 
  private enum TestProgress {

    FIRST_CAM, SECOND_CAM, DONE
  }
  /** Creates a new VisionTestCommand. */
  public VisionTestCommand(VisionSubsystem eyeSubsystem) {
    visionSubsystem = eyeSubsystem;
    addRequirements(eyeSubsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    progress = TestProgress.FIRST_CAM;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(progress == TestProgress.FIRST_CAM){
      AprilTag tag = visionSubsystem.cameraList.get(0).getBestTag(1);
      if(tag != null) {
        progress = TestProgress.SECOND_CAM;
        System.out.println("WE FOUMND THE FIRST(1) CIMERA!!!!!! :D c: :3");
        message = "first(1) appel tag funod :3";
      }
    }
    
    else if(progress == TestProgress.SECOND_CAM){
      AprilTag tag = visionSubsystem.cameraList.get(1).getBestTag(1);
      if(tag != null) {
        progress = TestProgress.DONE;
        System.out.println("we found the second(2) chameras :D");
        message = "Second(2) april tag foind :3";
       
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return progress == TestProgress.DONE;
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addStringProperty("message", ()-> message, null);
  }
  
}
