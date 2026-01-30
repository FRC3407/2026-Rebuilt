package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;

/**
 * A vision processing camera.
 */
public interface VisionCamera {

    public VisionPoseEstimate getEstimatedRobotPose();

    public AprilTag getBestTag(Integer... fiducialIDs);

    public boolean isConnected();

    public double getPrevTimeStamp();

    public void setPrevTimestamp(double timeInSeconds);

    public void setPipelineIndex(int pipelineIndex);
}
