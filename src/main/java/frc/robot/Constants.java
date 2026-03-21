// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2  * Math.PI; // radians per second

        // Chassis configuration

        public static final double kTrackWidth = Units.inchesToMeters(23.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(23.5);

        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 3;
        public static final int kRearLeftDrivingCanId = 5;
        public static final int kFrontRightDrivingCanId = 7;
        public static final int kRearRightDrivingCanId = 9;

        public static final int kFrontLeftTurningCanId = 2;
        public static final int kRearLeftTurningCanId = 4;
        public static final int kFrontRightTurningCanId = 6;
        public static final int kRearRightTurningCanId = 8;

        public static final boolean kGyroReversed = false;

        // SwerveDrivePoseEstimator configurations
        public static final Pose2d initialPoseMeters = Pose2d.kZero;
        public static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
        public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));

        public static final int navxCanId = 0;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 13;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
    }

    public static final class OIConstants {
        public static final int kLeftJoystickPort = 0;
        public static final int kRightJoystickPort = 1;
        public static final int kXboxControllerPort = 2;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class TargetConstants {
        public static final double targetProportional = 0.15;
        public static final double targetIntegral = 0;
        public static final double targetDerivative = 0.011;
        public static final double pointProportional = 0.15;
        public static final double pointIntegral = 0;
        public static final double pointDerivative = 0.011;
        public static final Translation2d Blue_hub = new Translation2d(4.625, 4.035);
        public static final Translation2d Red_hub = new Translation2d(16.54 - 4.625, 4.035);
        public static final Transform2d shooterTransform = new Transform2d(-0.2, 0, Rotation2d.fromDegrees(-90));
    }

    public static final class PathfindingConstants{
        public static final Pose2d Blue_hub_pose = new Pose2d(3.625, 4.035, new Rotation2d());
        public static final Pose2d Red_hub_pose = new Pose2d(16.54 - 3.625, 4.035, new Rotation2d(Math.PI));
        public static final PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)
        );
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxJerkMetersPerSecondCubed = 2.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class VortexMotorConstants {
        public static final double kFreeSpeedRpm = 6784;
    }

    public static final class VisionConstants {
        public static final AprilTagFields kFieldLayout = AprilTagFields.k2026RebuiltWelded;
        public static final double kMaxAmbiguity = 0.10;
    }

    public static final class ShooterConstants {
        public static final double kShooterWheelFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;

        public static final int kShooterCanId = 16; // vortex motor
        public static final int kSpindexerCanId = 13; // neo motor

        public static final double launcherHeight = Units.inchesToMeters(40); // approximate, get better measurements
        public static final double launchAngle = Math.atan2(4.5, 2.25); // also approximate, get better measurements
        public static final double hubHeight = Units.inchesToMeters(72);
        public static final double hubDiameter = Units.inchesToMeters(41.7); // MINIMUM diameter of the hub funnel
        public static final double hubRadius = hubDiameter / 2;
    }

    public static final class IntakeConstants {
        /** All Neo */
        public static final int kIntakeCanId = 11;
        public static final int kDeployLeftCanId = 10;
    }

    public static final class ShooterDataConstants {
        private static final Map<Double, Double> ballGroundData = new HashMap<>(); // RPM -> Distance when hit the floor in feet
        private static final Map<Double, Double> distanceShotData = new HashMap<>(); // RPM -> Distance between robot and HUB in inches

        static {
            // FILL IN WITH USEFUL DATA LATER!!!
            ballGroundData.put(0.1, 0.0);
            ballGroundData.put(0.2, 0.15);
            ballGroundData.put(0.3, 0.7);
            //ballGroundData.put(0.4, 0.0);
            ballGroundData.put(0.5, 3.9);
            ballGroundData.put(0.6, 4.7);
            ballGroundData.put(0.7, 6.4);
            ballGroundData.put(0.8, 7.6);
            ballGroundData.put(0.9, 9.0);
            ballGroundData.put(1.0, 10.0);
            ballGroundData.put(1.1, 13.0);
            ballGroundData.put(1.2, 14.0);

            distanceShotData.put(0.95, 85.0);
            distanceShotData.put(0.85, 73.0);
            distanceShotData.put(0.75, 67.0);
            distanceShotData.put(0.65, 35.0);
            distanceShotData.put(0.70, 32.0);
            distanceShotData.put(1.00, 105.0);
            distanceShotData.put(1.05, 105.0);
            distanceShotData.put(1.10, 105.0);
        }

        private static double velFromDist(double d) {
            final double g = 9.80665; // m/s^2
            final double numerator = g * d*d - ShooterConstants.launcherHeight;
            final double denominator = d * Math.sin(ShooterConstants.launchAngle) * Math.sin(ShooterConstants.launchAngle);

            return Math.sqrt(numerator / denominator);
        }

        // create a map with the same entries but with the outputs being in velocity rather than distance
        public static final Map<Double, Double> rpmToVelocity = ballGroundData.entrySet().stream()
            .collect(Collectors.toMap(
                a -> a.getKey() * 5500, // scaling multiplier
                a -> velFromDist(a.getValue())
            ));

        public static double trapezoidApproximation(Map<Double, Double> map, double velocity) {
            // get bounds for approximation!
            double lower = 0;
            double upper = 1e10;
            for (Double val : map.keySet()) {
                if (map.get(val) <= velocity) lower = Math.max(lower, val);
                if (map.get(val) >= velocity) upper = Math.min(upper, val);
            }

            final double lowerVal = map.get(lower);
            final double upperVal = map.get(upper);

            final double t = (velocity - lowerVal) / (upperVal - lowerVal);
            return lower + (upper - lower) * t;
        }
    }
}
