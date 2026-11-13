// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// edit, changed formating
package frc.robot.commands;


import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.vision.VisionCamera;

/** This test makes sure the vision is working by testing the cameras individually */
public class VisionTestCommand extends Command {
    private String message;
    private boolean cameraOneStatus;
    private boolean cameraTwoStatus;
    private boolean cameraOneIsOn;
    private boolean cameraTwoIsOn;
    private final VisionSubsystem visionSubsystem;
    private Timer timer;

    public VisionTestCommand(VisionSubsystem eyeSubsystem) {
        visionSubsystem = eyeSubsystem;
        addRequirements(eyeSubsystem);

    }

    @Override
    public void initialize() {
        cameraOneStatus = false;
        cameraTwoStatus = false;
        cameraOneIsOn = false;
        cameraTwoIsOn = false;
        timer.start();
    }

    @Override
    public void execute() {

        if(!cameraOneIsOn){
            if (visionSubsystem.cameraList.get(0).isConnected()){
                message = "camera one connected";
                cameraOneIsOn = true;
                }
        }
        if(!cameraTwoIsOn){
            if (visionSubsystem.cameraList.get(1).isConnected()){
                message = "camera two connected";
                cameraOneIsOn = true;
                }
        }
        if (!cameraOneStatus) {
            AprilTag tag = visionSubsystem.cameraList.get(0).getBestTag(1);
            if (tag != null) {
                System.out.println("WE FOUMND THE FIRST(1) CIMERA!!!!!! :D c: :3");
                message = "first(1) appel tag funod :3";
                cameraOneStatus = true;
            }
        }

        if (!cameraTwoStatus) {
            AprilTag tag = visionSubsystem.cameraList.get(1).getBestTag(1);
            if (tag != null) {
                System.out.println("we found the second(2) chameras :D");
                message = "Second(2) april tag foind :3";
                cameraTwoStatus = true;

            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        if(timer.hasElapsed(60)){
            message = "test timed out";
            return true;
        }
        if(cameraOneStatus && cameraTwoStatus){
            message = "both cameras good thumbs up";
           return true; 
        }
        return false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("message", () -> message, null);
        builder.addBooleanProperty("cameraOneIsOn", () -> cameraTwoIsOn, null);
        builder.addBooleanProperty("cameraTwoIsOn", () -> cameraTwoIsOn, null);
        builder.addBooleanProperty("cameraOneStatus", () -> cameraOneStatus, null);
        builder.addBooleanProperty("cameraTwoStatus", () -> cameraTwoStatus, null);

    }

}
