// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meter;
import static frc.robot.Constants.TargetConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Hub targeting command {@code DriveSubystem} using right joystick only */
public class TargetCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardStick;
    private final DoubleSupplier sidewaysStick;
    private final PIDController targetLockPID = new PIDController(targetProportional, targetIntegral, targetDerivative);
    private Rotation2d relative_rotation = new Rotation2d();
    /**
     * Drive the robot using joysticks.
     * 
     * @param forwardStick  Joystick for forward translation.
     * @param sidewaysStick Joystick for sideways translation.
     * @param rotStick      Joystick axis for rotation.
     * @param drive         DriveSubsystem
     */
    public TargetCommand(DoubleSupplier forwardStick, DoubleSupplier sidewaysStick, DriveSubsystem drive) {
        this.forwardStick = forwardStick;
        this.sidewaysStick = sidewaysStick;
        this.driveSubsystem = drive;
        addRequirements(this.driveSubsystem);
        SmartDashboard.putData("Target Command", this);
        }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Proportional", () -> targetLockPID.getP(), (p) -> targetLockPID.setP(p));
        builder.addDoubleProperty("Integral", () -> targetLockPID.getI(), (i) -> targetLockPID.setI(i));
        builder.addDoubleProperty("Derivative", () -> targetLockPID.getD(), (d) -> targetLockPID.setD(d));
        builder.addDoubleProperty("relative rotation to hub", () -> relative_rotation.getDegrees(), null);
    }
    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(forwardStick.getAsDouble() * Math.abs(forwardStick.getAsDouble()), OIConstants.kDriveDeadband) * -1;
        double ySpeed = MathUtil.applyDeadband(sidewaysStick.getAsDouble() * Math.abs(sidewaysStick.getAsDouble()), OIConstants.kDriveDeadband) * -1;
        Pose2d targetpose = new Pose2d(driveSubsystem.getAllianceRelative(Red_hub, Blue_hub), new Rotation2d());
        Pose2d currentpose = driveSubsystem.getPose();
        currentpose = currentpose.transformBy(shooterTransform);
        Rotation2d ang = currentpose.getRotation();
        Distance y = currentpose.getMeasureY();
        Distance x = currentpose.getMeasureX();
        Distance tagx = targetpose.getMeasureX();
        Distance tagy = targetpose.getMeasureY();
        Distance deltax = tagx.minus(x);
        Distance deltay = tagy.minus(y);
        Double ang_to_target = Math.atan2(deltay.in(Meter), deltax.in(Meter));
        Rotation2d angle_to_target_radians = new Rotation2d(ang_to_target);
        relative_rotation = ang.relativeTo(angle_to_target_radians);
        double rot = MathUtil.clamp(targetLockPID.calculate(relative_rotation.getRadians(), 0), -1, 1);
        //stop rotation when "close enough" doesn't do anything in teleop, since will be scheduled over by drive command when targetcommand ends, but in autonomous this prevents extra rotation. 
        if (Math.abs(relative_rotation.getDegrees()) > targetDeadbandDegrees){
            driveSubsystem.drive(xSpeed, ySpeed, rot, true);

        }
        else{
            driveSubsystem.drive(xSpeed, ySpeed, 0, true);
        }   
    }
    //ends command when within target deadband
    @Override
    public boolean isFinished() {
        if(relative_rotation != null){
            if(Math.abs(relative_rotation.getDegrees()) < targetDeadbandDegrees){
                return true;
            }   
            else {
                return false;
            }
        }
        else{
            return false;
        }
    }
}
