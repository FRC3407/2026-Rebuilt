package frc.robot.subsystems.vision;

import java.util.Collection;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Container for pose estimates.
 */
public class VisionPoseEstimate {

    public final Pose2d pose2d;
    public final double timestampSeconds;
    public final Collection<Integer> fiducialIDList;

    public VisionPoseEstimate(Pose2d pose2d, double timestampSeconds, Collection<Integer> idList) {
        this.pose2d = pose2d;
        this.timestampSeconds = timestampSeconds;
        this.fiducialIDList = idList;
    }

    public VisionPoseEstimate(EstimatedRobotPose estimatedRobotPose, Collection<Integer> idList) {
        this(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds, idList);
    }

    @Override
    public String toString() {
        return String.format("VisionPoseEstimate(%s, %.3f, %s)", this.pose2d, this.timestampSeconds, this.fiducialIDList);
    }
}
