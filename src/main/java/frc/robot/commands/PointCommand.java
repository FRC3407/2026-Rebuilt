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
    private final PIDController pointPID = new PIDController(pointProportional, pointIntegral, pointDerivative);

    /**
     * Drive the robot using joysticks.
     * 
     * @param forwardStick  Joystick for forward translation.
     * @param sidewaysStick Joystick for sideways translation.
     * @param rotStick      Joystick axis for rotation.
     * @param drive         DriveSubsystem
     */
    public PointCommand(DoubleSupplier forwardStick, DoubleSupplier sidewaysStick, DoubleSupplier rotStickx,
            DoubleSupplier rotSticky, DriveSubsystem drive) {
        this.forwardStick = forwardStick;
        this.sidewaysStick = sidewaysStick;
        this.rotStickx = rotStickx;
        this.rotSticky = rotSticky;
        this.driveSubsystem = drive;
        addRequirements(this.driveSubsystem);
        SmartDashboard.putData("PID", this);
    }

    private double joystickDifference() {
        double x = -rotStickx.getAsDouble();
        double y = rotSticky.getAsDouble();

        Translation2d j = new Translation2d(x, y);
        Rotation2d jAngle = j.getAngle();
        Rotation2d bAngle = driveSubsystem.getPose().getRotation();
        double jb = jAngle.minus(bAngle).getRadians();
        System.out.println(jb);
        return MathUtil.applyDeadband(jb, OIConstants.kDriveDeadband);
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
        double xSpeed = MathUtil.applyDeadband(forwardStick.getAsDouble() * Math.abs(forwardStick.getAsDouble()), OIConstants.kDriveDeadband) * -1;
        double ySpeed = MathUtil.applyDeadband(sidewaysStick.getAsDouble() * Math.abs(sidewaysStick.getAsDouble()), OIConstants.kDriveDeadband) * -1;
        double rot = MathUtil.clamp(pointPID.calculate(joystickDifference(), 0), -1, 1);
        driveSubsystem.drive(xSpeed, ySpeed, rot, true);
    }
}
