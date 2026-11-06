// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeeperSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BeeperTestCommand extends Command {
  private final BeeperSubsystem beeperSubsystem;
  private double elapsedTime;
  private final double BEEF_LENGTH = 2.0;
  private String message;

  /** Creates a new BeeperTestCommand. */
  public BeeperTestCommand(BeeperSubsystem funnyHonkHonk) {
    this.beeperSubsystem = funnyHonkHonk;
    addRequirements(this.beeperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    beeperSubsystem.beep(BEEF_LENGTH);
    elapsedTime = BEEF_LENGTH;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("status", ()->message, null);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(beeperSubsystem.getOutput()){
        elapsedTime -= 1/50;
      }

      else{
        // WYATT: take the uuuuhh *wyatt ponders for a very long time, then yaps to tony because his adhd is severe*
        // if the difference between the BEEF_LENGTH and the elapsedTime is too big (like 0.1 or something idk)
        // then we are going to error the test otherwise its good
        if(Math.abs(elapsedTime - BEEF_LENGTH)>0.1){
          //wyatt is complaining idk what hes complaining about though
          String msg = "BEEFER TEST FAILED";
          System.out.println(msg);
          this.message = msg;
        }
        else{
          //"then we are going to be succesful" - Wyatt Snickerwatt 8:35 pm 10/20/2026
          String msg = "BEEFER TEST SUCCEEDED!!!!";
          System.out.println(msg);
          this.message = msg;
        }
      }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
