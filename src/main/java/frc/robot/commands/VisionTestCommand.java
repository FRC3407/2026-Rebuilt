// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// edit, changed formating
package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

/** This test makes sure the vision is working by testing the cameras individually */
public class VisionTestCommand extends Command {
    private TestProgress progress;
    private String message;
    private final VisionSubsystem visionSubsystem;

    private enum TestProgress {

        FIRST_CAM, SECOND_CAM, DONE
    }

    public VisionTestCommand(VisionSubsystem eyeSubsystem) {
        visionSubsystem = eyeSubsystem;
        addRequirements(eyeSubsystem);

    }

    @Override
    public void initialize() {
        progress = TestProgress.FIRST_CAM;

    }

    @Override
    public void execute() {
        if (progress == TestProgress.FIRST_CAM) {
            AprilTag tag = visionSubsystem.cameraList.get(0).getBestTag(1);
            if (tag != null) {
                progress = TestProgress.SECOND_CAM;
                System.out.println("WE FOUMND THE FIRST(1) CIMERA!!!!!! :D c: :3");
                message = "first(1) appel tag funod :3";
            }
        }

        else if (progress == TestProgress.SECOND_CAM) {
            AprilTag tag = visionSubsystem.cameraList.get(1).getBestTag(1);
            if (tag != null) {
                progress = TestProgress.DONE;
                System.out.println("we found the second(2) chameras :D");
                message = "Second(2) april tag foind :3";

            }
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return progress == TestProgress.DONE;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("message", () -> message, null);
    }

}
