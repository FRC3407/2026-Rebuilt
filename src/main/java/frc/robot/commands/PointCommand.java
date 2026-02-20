// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meter;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.TargetConstants.*;

/** Hub targeting command {@code DriveSubystem} using right joystick only */
public class PointCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardStick;
    private final DoubleSupplier sidewaysStick;
    private final DoubleSupplier rotStickx;
    private final DoubleSupplier rotSticky;
    private final PIDController pointPID = new PIDController(Proportional, Integral, Derivative);


    /**
     * Drive the robot using joysticks.
     * 
     * @param forwardStick  Joystick for forward translation.
     * @param sidewaysStick Joystick for sideways translation.
     * @param rotStick      Joystick axis for rotation.
     * @param drive         DriveSubsystem
     */
    public PointCommand(DoubleSupplier forwardStick, DoubleSupplier sidewaysStick, DoubleSupplier rotStickx, DoubleSupplier rotSticky, DriveSubsystem drive) {
        this.forwardStick = forwardStick;
        this.sidewaysStick = sidewaysStick;
        this.rotStickx = rotStickx;
        this.rotSticky = rotSticky;
        this.driveSubsystem = drive;
        addRequirements(this.driveSubsystem);
        SmartDashboard.putData("PID", this);
    }

    private double joystickDifference() {
        double x = MathUtil.applyDeadband(rotStickx.getAsDouble(), OIConstants.kDriveDeadband);
        double y = MathUtil.applyDeadband(rotSticky.getAsDouble(), OIConstants.kDriveDeadband);
        Translation2d j = new Translation2d(x, y);
        double jAngle = j.getAngle().getRadians();
        double bAngle = driveSubsystem.getPose().getRotation().getRadians();
        double jb = jAngle - bAngle;
        double angle = MathUtil.angleModulus(jb);
        System.out.println(angle);

        return angle;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Proportional", () -> pointPID.getP(), (p) -> pointPID.setP(p));
        builder.addDoubleProperty("Integral", () -> pointPID.getI(), (i) -> pointPID.setI(i));
        builder.addDoubleProperty("Derivative", () -> pointPID.getD(), (d) -> pointPID.setD(d));
    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(forwardStick.getAsDouble(), OIConstants.kDriveDeadband) * -1;
        double ySpeed = MathUtil.applyDeadband(sidewaysStick.getAsDouble(), OIConstants.kDriveDeadband) * -1;
        if (RobotBase.isSimulation()) {
            ySpeed = MathUtil.applyDeadband(sidewaysStick.getAsDouble(), OIConstants.kDriveDeadband) * -1;
            xSpeed = MathUtil.applyDeadband(forwardStick.getAsDouble(), OIConstants.kDriveDeadband);
        }
        double rot = MathUtil.clamp(pointPID.calculate(joystickDifference(), 0), -1, 1);
        driveSubsystem.drive(xSpeed, ySpeed, rot, true);
    }
}
