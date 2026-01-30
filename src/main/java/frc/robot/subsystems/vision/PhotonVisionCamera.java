package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;

/**
 * Extends {@code PhotonCamera} to estimate poses and find visible
 * {@code AprilTag} Objects.
 */
public class PhotonVisionCamera implements VisionCamera {

    final PhotonCamera camera;
    final Transform3d cameraTransform;
    final PhotonPoseEstimator poseEstimator;
    private double prevTimestamp;
    protected final AprilTagFieldLayout tagLayout;

    public PhotonVisionCamera(String cameraName, AprilTagFieldLayout tgl, Transform3d cameraTransform) {
        this(new PhotonCamera(cameraName),
                tgl,
                cameraTransform,
                new PhotonPoseEstimator(tgl, cameraTransform));
    }

    PhotonVisionCamera(PhotonCamera camera, AprilTagFieldLayout tgl, Transform3d cameraTransform,
            PhotonPoseEstimator poseEstimator) {
        this.camera = camera;
        this.cameraTransform = cameraTransform;
        this.poseEstimator = poseEstimator;
        this.tagLayout = tgl;
    }

    @Override
    public double getPrevTimeStamp() {
        return prevTimestamp;
    }

    @Override
    public void setPrevTimestamp(double timeInSeconds) {
        this.prevTimestamp = timeInSeconds;
    }

    /**
     * @return the estimated field-centric pose of the robot, or
     *         {@code null}.
     */
    @Override
    public VisionPoseEstimate getEstimatedRobotPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult pipelineResult : camera.getAllUnreadResults()) {
            List<PhotonTrackedTarget> targetList = makeGoodTargetList(pipelineResult, null);
            if (!targetList.isEmpty()) {
                visionEst = estimateBestPose(pipelineResult);
            }
        }
        return visionEst.isPresent() ? new VisionPoseEstimate(visionEst.get()) : null;
    }

    private Optional<EstimatedRobotPose> estimateBestPose(PhotonPipelineResult pipelineResult) {
        if (!pipelineResult.getMultiTagResult().isEmpty()) {
            return poseEstimator.estimateCoprocMultiTagPose(pipelineResult);
        }
        return poseEstimator.estimateLowestAmbiguityPose(pipelineResult);
    }

    /**
     * @return True if the camera is actively sending frame data, false
     *         otherwise.
     */
    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }

    /**
     * @return the best visible {@code AprilTag} for any of the fiducialIDs.
     */
    @Override
    public AprilTag getBestTag(Integer... fiducialIDs) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results == null || results.isEmpty()) {
            return null;
        }
        return getBestTag(results.get(0), fiducialIDs);
    }

    /**
     * @return the best {@code AprilTag} or {@code null}.
     */
    private AprilTag getBestTag(PhotonPipelineResult pipelineResult, Integer... fiducialIDs) {
        if (!pipelineResult.hasTargets()) {
            return null;
        }
        List<PhotonTrackedTarget> targetList = makeGoodTargetList(pipelineResult,
                (t1, t2) -> Double.compare(t1.poseAmbiguity, t2.poseAmbiguity));
        for (PhotonTrackedTarget target : targetList) {
            if (isFiducialListMatch(target, fiducialIDs)) {
                return toAprilTag(pipelineResult, target);
            }
        }
        return null;
    }

    /**
     * @return whether the observed target is on a list of IDs. If the ID
     *         list is empty, then match any target.
     */
    private boolean isFiducialListMatch(PhotonTrackedTarget target, Integer... fiducialIDs) {
        if (fiducialIDs == null || fiducialIDs.length == 0) {
            return true;
        }
        for (Integer id : fiducialIDs) {
            if (target.getFiducialId() == id) {
                return true;
            }
        }
        return false;
    }

    /**
     * Convert a pipeline result and target to an {@code AprilTag} object
     * with a field-relative {@code Pose3d}.
     */
    private AprilTag toAprilTag(PhotonPipelineResult pipelineResult, PhotonTrackedTarget target) {
        Optional<EstimatedRobotPose> visionEst = estimateBestPose(pipelineResult);
        if (!visionEst.isPresent()) {
            return null;
        }
        return makeAprilTag(target.getFiducialId(),
                visionEst.get().estimatedPose,
                target.getBestCameraToTarget());
    }

    /**
     * @return an {@code AprilTag} with a field-centric pose.
     */
    AprilTag makeAprilTag(int fiducialId, Pose3d fieldToRobotPose, Transform3d robotToTag) {
        Pose3d aprilTagPose = tagLayout.getTagPose(fiducialId)
                .orElse(fieldToRobotPose != null ? fieldToRobotPose.transformBy(robotToTag) : null);
        return new AprilTag(fiducialId, aprilTagPose);
    }

    /**
     * @return whether this target should be excluded from vision
     *         processing.
     */
    private boolean isBadTarget(PhotonTrackedTarget target) {
        return target.getPoseAmbiguity() > VisionConstants.kMaxAmbiguity;
    }

    /**
     * @return List of all {@code PhotonTrackedTarget} objects that are not
     *         "bad".
     */
    List<PhotonTrackedTarget> makeGoodTargetList(PhotonPipelineResult pipelineResult,
            Comparator<PhotonTrackedTarget> targetComparator) {
        List<PhotonTrackedTarget> targetList = new ArrayList<>(pipelineResult.getTargets());
        List<PhotonTrackedTarget> badTargets = pipelineResult.getTargets().stream()
                .filter(this::isBadTarget).toList();
        targetList.removeAll(badTargets);
        if (!targetList.isEmpty() && targetComparator != null) {
            Collections.sort(targetList, targetComparator);
        }
        return targetList;
    }

    @Override
    public void setPipelineIndex(int pipelineIndex) {
        camera.setPipelineIndex(pipelineIndex);
    }

    @Override
    public String toString() {
        return "PhotonVisionCamera[" + this.camera + "]";
    }
}
