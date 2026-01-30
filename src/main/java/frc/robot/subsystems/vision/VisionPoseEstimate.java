package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Container for pose estimates.
 */
public class VisionPoseEstimate {
    public final Pose2d pose2d;
    public final double timestampSeconds;

    public VisionPoseEstimate(Pose2d pose2d, double timestampSeconds) {
        this.pose2d = pose2d;
        this.timestampSeconds = timestampSeconds;
    }

    public VisionPoseEstimate(EstimatedRobotPose estimatedRobotPose) {
        this(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
    }
}
