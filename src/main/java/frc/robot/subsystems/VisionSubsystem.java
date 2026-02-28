package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.*;
import frc.robot.util.DebouncedBoolean;

/**
 * Interfaces with PhotonVision, updating the {@code DriveSubsystem} odometry
 * whenever an AprilTag is seen. Also allows Commands to determine if specific
 * AprilTags are visible.
 */
public class VisionSubsystem extends SubsystemBase {

    private final DebouncedBoolean isTargetVisible = new DebouncedBoolean(200L);
    private final DriveSubsystem driveSubsystem;
    public final NetworkTableEntry apriltagsVisibleDisplay = SmartDashboard.getEntry("VisionSubsystem/AprilTagsVisible");
    public final NetworkTableEntry periodicTimeDisplay = SmartDashboard.getEntry("VisionSubsystem/periodicTime");
    public final StringPublisher apriltagLogger = NetworkTableInstance.getDefault().getStringTopic("VisionSubsystem/periodicTime").publish();
    public final List<VisionCamera> cameraList = new ArrayList<>();
    protected final AprilTagFieldLayout tagLayout;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.tagLayout = AprilTagFieldLayout.loadField(VisionConstants.kFieldLayout);

        // Portside
        cameraList.add(new PhotonVisionCamera("Arducam_OV9281_USB_Camera", this.tagLayout, new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(-12.0), // left or right from center
                        Units.inchesToMeters(-6.0), // forward from robot center
                        Units.inchesToMeters(20.5)), // distance up from the floor
                new Rotation3d(
                        Rotation2d.fromDegrees(0).getRadians(), // roll
                        Rotation2d.fromDegrees(0.0).getRadians(), // pitch
                        Rotation2d.fromDegrees(90.0).getRadians())))); // yaw
        
        // Starboard camera
        cameraList.add(new PhotonVisionCamera("Arducam_OV9281_USB_Camera (1)", this.tagLayout, new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(12.0), // left or right from center
                        Units.inchesToMeters(-6.0), // forward from robot center
                        Units.inchesToMeters(20.5)), // distance up from the floor
                new Rotation3d(
                        Rotation2d.fromDegrees(0).getRadians(), // roll
                        Rotation2d.fromDegrees(0.0).getRadians(), // pitch
                        Rotation2d.fromDegrees(-90.0).getRadians())))); // yaw
                        
    }

    @Override
    public void periodic() {
        final double time = Timer.getFPGATimestamp();
        boolean targetFound = false;
        for (VisionCamera visionCamera : cameraList) {
            VisionPoseEstimate estimatedRobotPose = visionCamera.getEstimatedRobotPose();
            if (estimatedRobotPose != null) {
                targetFound = true;
                if (estimatedRobotPose.timestampSeconds != visionCamera.getPrevTimeStamp()) {
                    driveSubsystem.addVisionMeasurement(
                            estimatedRobotPose.pose2d,
                            estimatedRobotPose.timestampSeconds);
                    apriltagLogger.set(visionCamera.toString());
                }
                visionCamera.setPrevTimestamp(estimatedRobotPose.timestampSeconds);
            }
        }
        isTargetVisible.set(targetFound);
        apriltagsVisibleDisplay.setBoolean(isTargetVisible.get());
        periodicTimeDisplay.setInteger(Math.round(1000 * (Timer.getFPGATimestamp() - time)));
    }

    /**
     * @return whether any AprilTag is visible.
     */
    public boolean cameraSeesTargets() {
        return isTargetVisible.get();
    }

    /**
     * Find the best {@code AprilTag} currently visible for a given list of IDs.
     * If no fiducial IDs are specified, then return any possible tag.
     *
     * @param fiducialIDs optional list of tag IDs.
     * @return the best visible {@code AprilTag} for any of the fiducialIDs.
     */
    public AprilTag getBestTag(Integer... fiducialIDs) {
        AprilTag aprilTag = null;
        for (VisionCamera visionCamera : cameraList) {
            aprilTag = visionCamera.getBestTag(fiducialIDs);
            if (aprilTag != null) {
                break;
            }
        }
        return aprilTag;
    }
}
