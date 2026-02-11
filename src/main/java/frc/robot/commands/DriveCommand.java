// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meter;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Default command for driving the {@code DriveSubystem} using joysticks */
public class DriveCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardStick;
    private final DoubleSupplier sidewaysStick;
    private final DoubleSupplier rotStick;
    private final BooleanSupplier targeting_switch;
    private final Translation2d R_hub = new Translation2d(4.02844, 3.522);
    private final Translation2d B_hub = new Translation2d(16.54 - 4.02844, 8.07 - 3.522);
    private final PIDController targetLockPID = new PIDController(2, 0, 0);

    /**
     * Drive the robot using joysticks.
     * 
     * @param forwardStick Joystick for forward translation.
     * @param sidewaysStick Joystick for sideways translation.
     * @param rotStick Joystick axis for rotation.
     * @param drive DriveSubsystem
     * @param targeting_switch Controls the targeting system.
     */
    public DriveCommand(DoubleSupplier forwardStick, DoubleSupplier sidewaysStick, DoubleSupplier rotStick, BooleanSupplier targeting_switch,
            DriveSubsystem drive) {
        this.forwardStick = forwardStick;
        this.sidewaysStick = sidewaysStick;
        this.rotStick = rotStick;
        this.driveSubsystem = drive;
        this.targeting_switch = targeting_switch;
        addRequirements(this.driveSubsystem);
    }

    private Translation2d getTargetHub() {
        Optional<Alliance> al = DriverStation.getAlliance();
        if(al.get() == DriverStation.Alliance.Blue){
            return R_hub;
        }
        else{
            return B_hub;
        }
    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(sidewaysStick.getAsDouble(), OIConstants.kDriveDeadband) * -1;
        double ySpeed = MathUtil.applyDeadband(forwardStick.getAsDouble(), OIConstants.kDriveDeadband);
        double rot = MathUtil.applyDeadband(rotStick.getAsDouble(), OIConstants.kDriveDeadband) * -1;
        if (RobotBase.isSimulation()){
            ySpeed = MathUtil.applyDeadband(sidewaysStick.getAsDouble(), OIConstants.kDriveDeadband) * -1;
            xSpeed = MathUtil.applyDeadband(forwardStick.getAsDouble(), OIConstants.kDriveDeadband);
            rot = MathUtil.applyDeadband(rotStick.getAsDouble(), OIConstants.kDriveDeadband);
        }
        if (targeting_switch.getAsBoolean()) {
            System.out.println("Left Bumper Pressed"); 
            //change commented line to go from april tag to coordinate lock
            // Pose2d targetpose = getTargetTagPose();      
            Pose2d targetpose = new Pose2d(getTargetHub(), new Rotation2d());
            if (targetpose != null) {
                Pose2d currentpose = driveSubsystem.getPose();
                Rotation2d ang = currentpose.getRotation();
                Distance y = currentpose.getMeasureY();
                Distance x = currentpose.getMeasureX();
                Distance tagx = targetpose.getMeasureX();
                Distance tagy = targetpose.getMeasureY();
                Distance deltax = tagx.minus(x);
                Distance deltay = tagy.minus(y);
                Double ang_to_target = Math.atan2(deltay.in(Meter), deltax.in(Meter));
                Rotation2d angle_to_target_radians = new Rotation2d(ang_to_target);
                Rotation2d relative_rotation = ang.relativeTo(angle_to_target_radians);
                double raw_rot = -targetLockPID.calculate(relative_rotation.getRadians(), 0);
                System.out.print(raw_rot);
                if (raw_rot < -1){
                    rot = -1;
                }
                if (raw_rot > 1){
                    rot = 1;
                }
                else{
                    rot = raw_rot;
                }
            }
        }
        rot = rot * -1;
                driveSubsystem.drive(xSpeed, ySpeed, rot, true);
    }
}
