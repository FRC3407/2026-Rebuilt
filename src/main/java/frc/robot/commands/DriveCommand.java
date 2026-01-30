// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Default command for driving the {@code DriveSubystem} using joysticks */
public class DriveCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardStick;
    private final DoubleSupplier sidewaysStick;
    private final DoubleSupplier rotStick;

    /**
     * Drive the robot using joysticks.
     * 
     * @param forwardStick Joystick for forward translation.
     * @param sidewaysStick Joystick for sideways translation.
     * @param rotStick Joystick axis for rotation.
     * @param drive DriveSubsystem
     */
    public DriveCommand(DoubleSupplier forwardStick, DoubleSupplier sidewaysStick, DoubleSupplier rotStick,
            DriveSubsystem drive) {
        this.forwardStick = forwardStick;
        this.sidewaysStick = sidewaysStick;
        this.rotStick = rotStick;
        this.driveSubsystem = drive;
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(sidewaysStick.getAsDouble(), OIConstants.kDriveDeadband) * -1;
        double ySpeed = MathUtil.applyDeadband(forwardStick.getAsDouble(), OIConstants.kDriveDeadband);
        double rot = MathUtil.applyDeadband(rotStick.getAsDouble(), OIConstants.kDriveDeadband) * -1;
        driveSubsystem.drive(xSpeed, ySpeed, rot, true);
    }
}
