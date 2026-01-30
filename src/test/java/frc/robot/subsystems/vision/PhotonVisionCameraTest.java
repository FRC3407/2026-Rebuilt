package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PhotonVisionCameraTest {

    static RobotContainer robotContainer;
    static DriveSubsystem driveSubsystem;
    static VisionSubsystem visionSubsystem;
    static Transform3d cameraTransform;
    static AprilTagFieldLayout tagLayout;

    @BeforeAll
    static void setup() {
        tagLayout = AprilTagFieldLayout.loadField(VisionConstants.kFieldLayout);
        robotContainer = RobotContainer.getInstance();
        driveSubsystem = robotContainer.m_robotDrive;
        visionSubsystem = new VisionSubsystem(driveSubsystem);
        visionSubsystem.cameraList.clear(); // Clear camera list for this test

        cameraTransform = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(0.0), // left or right from center
                        Units.inchesToMeters(8.0), // forward from robot center
                        Units.inchesToMeters(12.0)), // distance up from the floor
                new Rotation3d(
                        Rotation2d.fromDegrees(0).getRadians(), // roll
                        Rotation2d.fromDegrees(0.0).getRadians(), // pitch
                        Rotation2d.fromDegrees(0).getRadians())); // yaw
    }

    @Test
    void testInitialization() {
        PhotonVisionCamera visionCamera = new PhotonVisionCamera("TestCamera", tagLayout, cameraTransform);
        assertNotNull(visionCamera);
        assertNotNull(visionCamera.camera);
        assertNotNull(visionCamera.poseEstimator);
    }

    @Test
    void testIsConnected() {
        PhotonVisionCamera visionCamera = new PhotonVisionCamera("TestCamera", tagLayout, cameraTransform);
        assertFalse(visionCamera.isConnected());
    }

    @Test
    void testGetBestTag_Subsystem_WithNoMatchingIDs() {
        // Arrange
        visionSubsystem.cameraList.clear();

        PhotonVisionCamera mockCamera = mock(PhotonVisionCamera.class);
        visionSubsystem.cameraList.add(mockCamera);

        when(mockCamera.getBestTag(4, 5, 6)).thenReturn(null);

        // Act
        AprilTag result = visionSubsystem.getBestTag(4, 5, 6);

        // Assert
        assertNull(result);
    }

    @Test
    void testGetBestTag_Subsystem_WithMatchingFiducialID() {
        // Arrange
        visionSubsystem.cameraList.clear();

        PhotonVisionCamera mockCamera = mock(PhotonVisionCamera.class);
        visionSubsystem.cameraList.add(mockCamera);

        int fiducialId = 3;
        AprilTag mockTag = new AprilTag(fiducialId, new Pose3d());
        when(mockCamera.getBestTag()).thenReturn(mockTag);

        // Act
        AprilTag result = visionSubsystem.getBestTag(); // Empty list, so match any tag

        // Assert
        assertNotNull(result);
        assertEquals(fiducialId, result.ID);
    }

    @Test
    void testGetEstimatedRobotPose_WithNoResults() {
        // Arrange
        PhotonCamera mockCamera = mock(PhotonCamera.class);
        PhotonPoseEstimator mockEstimator = mock(PhotonPoseEstimator.class);
        PhotonVisionCamera visionCamera = new PhotonVisionCamera(mockCamera, tagLayout, new Transform3d(),
                mockEstimator);

        when(mockCamera.getAllUnreadResults()).thenReturn(Collections.emptyList());

        // Act
        VisionPoseEstimate pose = visionCamera.getEstimatedRobotPose();

        // Assert
        assertNull(pose, "Pose should be null when no results are returned");
    }

    @Test
    void testGetEstimatedRobotPose_NoTargets() {
        // Arrange
        PhotonCamera mockCamera = mock(PhotonCamera.class);
        PhotonPoseEstimator mockEstimator = mock(PhotonPoseEstimator.class);
        PhotonVisionCamera visionCamera = new PhotonVisionCamera(mockCamera, tagLayout, new Transform3d(),
                mockEstimator);

        PhotonPipelineResult pipelineResult = mock(PhotonPipelineResult.class);
        when(pipelineResult.hasTargets()).thenReturn(false);
        when(pipelineResult.getTargets()).thenReturn(List.of());

        when(mockCamera.getAllUnreadResults()).thenReturn(List.of(pipelineResult));

        // Act
        VisionPoseEstimate pose = visionCamera.getEstimatedRobotPose();

        // Assert
        assertNull(pose, "Pose should be null when no targets are detected.");
    }

    @Test
    void testGetEstimatedRobotPose_WithTargets() {
        // Arrange
        PhotonVisionCamera visionCamera = new PhotonVisionCamera("TestCamera", tagLayout, new Transform3d()) {
            @Override
            public VisionPoseEstimate getEstimatedRobotPose() {
                Pose3d pose3d = new Pose3d();
                EstimatedRobotPose visionEst = new EstimatedRobotPose(pose3d, 0.0, new ArrayList<PhotonTrackedTarget>(),
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR); // Simulate a valid pose
                return new VisionPoseEstimate(visionEst);
            }
        };

        // Act
        VisionPoseEstimate pose = visionCamera.getEstimatedRobotPose();

        // Assert
        assertNotNull(pose, "Pose should not be null when targets are detected.");
    }

    @Test
    void testMakeAprilTag() {
        // Arrange
        PhotonVisionCamera visionCamera = new PhotonVisionCamera(null, tagLayout, cameraTransform, null);
        int fiducialId = 3;
        Pose3d fieldToRobotPose = new Pose3d(
                new Translation3d(4.0, 3.0, 0.0),
                new Rotation3d(0.0, 0.0, 0.0));
        Transform3d robotToTag = new Transform3d(new Transform2d(
                2.0, 1.0, new Rotation2d(0.0)));

        // Act
        AprilTag aprilTag = visionCamera.makeAprilTag(fiducialId, fieldToRobotPose, robotToTag);

        // Assert
        assertEquals(11.31, aprilTag.pose.getX(), 0.05);
        assertEquals(4.39, aprilTag.pose.getY(), 0.05);
        assertEquals(1.12, aprilTag.pose.getZ(), 0.05);
    }

    @Test
    void testMakeGoodTargetList_FiltersBadTargets() {
        // Arrange
        PhotonVisionCamera visionCamera = new PhotonVisionCamera("TestCamera", tagLayout, new Transform3d());

        PhotonPipelineResult pipelineResult = mock(PhotonPipelineResult.class);
        PhotonTrackedTarget goodTarget = mock(PhotonTrackedTarget.class);
        PhotonTrackedTarget badTarget = mock(PhotonTrackedTarget.class);

        when(goodTarget.getPoseAmbiguity()).thenReturn(0.10);
        when(badTarget.getPoseAmbiguity()).thenReturn(0.90);
        when(pipelineResult.getTargets()).thenReturn(List.of(goodTarget, badTarget));

        // Act
        List<PhotonTrackedTarget> goodTargets = visionCamera.makeGoodTargetList(pipelineResult, null);

        // Assert
        assertEquals(1, goodTargets.size());
        assertEquals(goodTarget, goodTargets.get(0));
    }

    @Test
    void testMakeGoodTargetList_SortsTargets() {
        // Arrange
        PhotonVisionCamera visionCamera = new PhotonVisionCamera("TestCamera", tagLayout, new Transform3d());

        PhotonPipelineResult pipelineResult = mock(PhotonPipelineResult.class);
        PhotonTrackedTarget target1 = mock(PhotonTrackedTarget.class);
        PhotonTrackedTarget target2 = mock(PhotonTrackedTarget.class);

        when(target1.getPoseAmbiguity()).thenReturn(0.05);
        when(target2.getPoseAmbiguity()).thenReturn(0.02);
        when(pipelineResult.getTargets()).thenReturn(List.of(target1, target2));

        // Act
        List<PhotonTrackedTarget> sortedTargets = visionCamera.makeGoodTargetList(pipelineResult,
                Comparator.comparingDouble(PhotonTrackedTarget::getPoseAmbiguity));

        // Assert
        assertEquals(2, sortedTargets.size());
        assertEquals(target2, sortedTargets.get(0));
        assertEquals(target1, sortedTargets.get(1));
    }

    @Test
    void testMakeGoodTargetList_ManyTargets() {
        // Arrange
        PhotonVisionCamera visionCamera = new PhotonVisionCamera(null, tagLayout, cameraTransform, null);
        PhotonPipelineResult pipelineResult = new PhotonPipelineResult();
        int size = 8;
        pipelineResult.targets = new ArrayList<PhotonTrackedTarget>();
        for (int i = 0; i < size; i++) {
            PhotonTrackedTarget target = mock(PhotonTrackedTarget.class);
            final double ambiguity = (i % 2 == 0) ? 0.0 : Constants.VisionConstants.kMaxAmbiguity + 0.1 * (size - i);
            when(target.getPoseAmbiguity()).thenReturn(ambiguity);
            pipelineResult.targets.add(target);
        }

        assertTrue(pipelineResult.hasTargets());
        assertEquals(size, pipelineResult.getTargets().size());

        // Act
        List<PhotonTrackedTarget> targetList = visionCamera.makeGoodTargetList(pipelineResult, null);

        // Assert
        assertEquals((size + 1) / 2, targetList.size());

        // Act again, this time sorting list
        targetList = visionCamera.makeGoodTargetList(pipelineResult,
                (t1, t2) -> Double.compare(t1.poseAmbiguity, t2.poseAmbiguity));

        // Assert again
        assertEquals((size + 1) / 2, targetList.size());
    }

    @Test
    void testMakeGoodTargetList_EmptyWhenNoTargets() {
        // Arrange
        PhotonVisionCamera visionCamera = new PhotonVisionCamera("TestCamera", tagLayout, new Transform3d());

        PhotonPipelineResult pipelineResult = mock(PhotonPipelineResult.class);
        when(pipelineResult.getTargets()).thenReturn(List.of());

        // Act
        List<PhotonTrackedTarget> goodTargets = visionCamera.makeGoodTargetList(pipelineResult, null);

        // Assert
        assertEquals(0, goodTargets.size());
    }

    @Test
    void testGetBestTag_WithNoResults() {
        // Arrange
        PhotonCamera mockCamera = mock(PhotonCamera.class);
        PhotonPoseEstimator mockEstimator = mock(PhotonPoseEstimator.class);
        PhotonVisionCamera visionCamera = new PhotonVisionCamera(mockCamera, tagLayout, new Transform3d(),
                mockEstimator);

        when(mockCamera.getAllUnreadResults()).thenReturn(Collections.emptyList());

        // Act
        AprilTag result = visionCamera.getBestTag(4, 5, 6);

        // Assert
        assertNull(result);
    }

    @Test
    void testGetBestTag_NoTargets() {
        // Arrange
        PhotonCamera mockCamera = mock(PhotonCamera.class);
        PhotonPoseEstimator mockEstimator = mock(PhotonPoseEstimator.class);
        PhotonVisionCamera visionCamera = new PhotonVisionCamera(mockCamera, tagLayout, new Transform3d(),
                mockEstimator);

        PhotonPipelineResult pipelineResult = mock(PhotonPipelineResult.class);
        when(pipelineResult.hasTargets()).thenReturn(false);
        when(pipelineResult.getTargets()).thenReturn(List.of());

        when(mockCamera.getAllUnreadResults()).thenReturn(List.of(pipelineResult));

        // Act
        AprilTag result = visionCamera.getBestTag(4, 5, 6);

        // Assert
        assertNull(result);
    }

    @Test
    void testGetBestTag_NoMatchingTargets() {
        // Arrange
        PhotonCamera mockCamera = mock(PhotonCamera.class);
        PhotonPoseEstimator mockEstimator = mock(PhotonPoseEstimator.class);
        PhotonVisionCamera visionCamera = new PhotonVisionCamera(mockCamera, tagLayout, new Transform3d(),
                mockEstimator);

        PhotonTrackedTarget target1 = mock(PhotonTrackedTarget.class);
        when(target1.getFiducialId()).thenReturn(1);
        PhotonTrackedTarget target2 = mock(PhotonTrackedTarget.class);
        when(target1.getFiducialId()).thenReturn(2);

        PhotonPipelineResult pipelineResult = mock(PhotonPipelineResult.class);
        when(pipelineResult.hasTargets()).thenReturn(true);
        when(pipelineResult.getTargets()).thenReturn(List.of(target1, target2));

        when(mockCamera.getAllUnreadResults()).thenReturn(List.of(pipelineResult));

        // Act
        AprilTag result = visionCamera.getBestTag(4, 5, 6);

        // Assert
        assertNull(result);
    }

    @Test
    void testGetBestTag_MatchingTargets() {
        // Arrange
        PhotonCamera mockCamera = mock(PhotonCamera.class);
        PhotonPoseEstimator mockEstimator = mock(PhotonPoseEstimator.class);
        PhotonVisionCamera visionCamera = new PhotonVisionCamera(mockCamera, tagLayout, new Transform3d(),
                mockEstimator);

        PhotonTrackedTarget target1 = mock(PhotonTrackedTarget.class);
        when(target1.getFiducialId()).thenReturn(1);
        PhotonTrackedTarget target2 = mock(PhotonTrackedTarget.class);
        when(target2.getFiducialId()).thenReturn(2);
        PhotonTrackedTarget target3 = mock(PhotonTrackedTarget.class);
        when(target3.getFiducialId()).thenReturn(3);

        PhotonPipelineResult pipelineResult = mock(PhotonPipelineResult.class);
        when(pipelineResult.hasTargets()).thenReturn(true);
        when(pipelineResult.getTargets()).thenReturn(List.of(target1, target2, target3));

        when(mockCamera.getAllUnreadResults()).thenReturn(List.of(pipelineResult));

        EstimatedRobotPose mockPose = mock(EstimatedRobotPose.class);
        when(target2.getBestCameraToTarget()).thenReturn(new Transform3d());
        when(mockEstimator.estimateLowestAmbiguityPose(pipelineResult)).thenReturn(Optional.of(mockPose));

        // Act
        AprilTag result = visionCamera.getBestTag(8, 2, 6);

        // Assert
        assertNotNull(result);
        assertEquals(2, result.ID);
    }
}