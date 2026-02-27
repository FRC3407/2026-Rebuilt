// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import java.util.Optional;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.studica.frc.Navx;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.Constants.TargetConstants;
import frc.robot.RobotContainer;
import static frc.robot.Constants.TargetConstants.*;

public class DriveSubsystem extends SubsystemBase {

    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    // The gyro sensor
    private final Navx m_gyro = new Navx(DriveConstants.navxCanId);

    // Odometry class for tracking robot pose
    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            },
            DriveConstants.initialPoseMeters,
            DriveConstants.stateStdDevs,
            DriveConstants.visionMeasurementStdDevs);

    // config variable for pathplanner
    private final RobotConfig config;
    /**
     * SmartDashboard widget for displaying gyro angle.
     */
    public final NetworkTableEntry gyroDisplay = SmartDashboard.getEntry("DriveSubsystem/Gyro");

    /**
     * SmartDashboard widget for displaying the battery voltage.
     */
    public final NetworkTableEntry voltageDisplay = SmartDashboard.getEntry("PowerDistribution/Voltage");
    private final PowerDistribution powerDistribution = new PowerDistribution();

    /**
     * SmartDashboard widget for displaying robot location.
     */
    public final Field2d odometryDisplay = new Field2d();

    /**
     * Logger for putting robot location into AdvantageScope.
     */
    private final StructPublisher<Pose2d> odometryLogger;

    // Simulated Gryo
    private Rotation2d m_simGyroAngle = Rotation2d.fromDegrees(0);
    private double m_simGyroRate;

    public DriveSubsystem() {
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

        SmartDashboard.putData(odometryDisplay);
        odometryLogger = NetworkTableInstance.getDefault().getStructTopic("Odometry", Pose2d.struct).publish();
        config = loadPathPlannerSettings();
        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getChassisSpeeds,
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)),
                config,
                this::isRedAlliance,
                this);
    }

    private boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    private RobotConfig loadPathPlannerSettings() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    @Override
    public void periodic() {
        final Pose2d updatedPose = m_odometry.update(
                Rotation2d.fromDegrees(getHeading()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });

        gyroDisplay.setNumber(getHeading());
        // voltageDisplay.setNumber(powerDistribution.getVoltage());
        odometryDisplay.setRobotPose(updatedPose);
        odometryLogger.set(getPose());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }

    public <T> T getAllianceRelative(T red_var, T blue_var){ //this needs to be fundamentally different from gettarget hub, as it should be able to handle a variety of 2d types and a variety of variables
        if (isRedAlliance()){
            return red_var;
        }
        else{
            return blue_var;
        }
    }
    public Supplier<Command> pathfindToPoseSupplier(Pose2d red_pose, Pose2d blue_pose){
        Supplier<Command> poseCommandSupplier = ()-> AutoBuilder.pathfindToPose(
            getAllianceRelative(red_pose, blue_pose),
            PathfindingConstants.constraints,
            0.0
        );
        return poseCommandSupplier;
    }

   /*  public Translation2d getTargetHub() {
        if (isRedAlliance()) {
            return Red_hub;
        } else {
            return Blue_hub;
        }
    } */ // kept this in case we want it, but might be repetitve considering getAllianceRelative

    /** @return distance in meters*/
    public double distanceToHub(){
        Pose2d currentpose = getPose();
        double distance = currentpose.getTranslation().getDistance(getAllianceRelative(Red_hub, Blue_hub));
        return distance;
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(getHeading()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        m_odometry.addVisionMeasurement(pose, timestamp);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to
     *                      the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(getHeading()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setSwerveModulesToX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        m_gyro.resetYaw();
        RobotContainer.getInstance().m_beeper.beep(0.5);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        if (RobotBase.isReal()) {
            return m_gyro.getYaw().magnitude();
        } else {
            return m_simGyroAngle.getDegrees();
        }
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        AngularVelocity[] omega = m_gyro.getAngularVel();
        return omega[2].in(DegreesPerSecond) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * @return chassis speeds of robot
     */
    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds speeds = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());

        return speeds;
    }

    /**
     * Method that will drive the robot Robot-Relative.
     */
    public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {// not currently using feed
        // forwards
        this.drive(speeds.vxMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond,
                speeds.vyMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond,
                speeds.omegaRadiansPerSecond / Constants.DriveConstants.kMaxAngularSpeed,
                false); //should NOT drive the robot field relative, as it is used in pathplanner as a way to drive the robot relative to the robot, hence driveRobotRelative, and not driveFieldRelative. Using field relative here will break pathplanner in paths with rotation and translation
    }

    // This can only be called from simulationPeriodic()
    private void updateGyro() {
        ChassisSpeeds speeds = getChassisSpeeds();
        m_simGyroRate = speeds.omegaRadiansPerSecond;
        m_simGyroAngle = m_simGyroAngle.plus(Rotation2d.fromRadians(m_simGyroRate * .02));
    }

    // This can only be called from simulationPeriodic()
    private void updateSimEncoders(MAXSwerveModule... swerveModules) {
        for (var swerveModule : swerveModules) {
            swerveModule.updateSimEncoders();
        }
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        updateSimEncoders(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight);
        updateGyro();
    }

}
