package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.PhotonVisionCamera;

public class VisionSubsystemTest {

    static RobotContainer robotContainer;
    static DriveSubsystem driveSubsystem;

    @BeforeAll
    static void setup() {
        robotContainer = RobotContainer.getInstance();
        driveSubsystem = robotContainer.m_robotDrive;
    }

    @Test
    void testInitialization() {
        VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);
        assertNotNull(visionSubsystem);
        visionSubsystem.cameraList.clear(); // Clear camera list for test
        assertEquals(visionSubsystem.cameraList.size(), 0);
    }

    @Test
    void testCameraSeesTargets() {
        VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);
        visionSubsystem.cameraList.clear(); // Clear camera list for this test

        visionSubsystem.periodic();

        assertFalse(visionSubsystem.cameraSeesTargets());
    }

    @Test
    void testGetBestTag() {
        VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);
        visionSubsystem.cameraList.clear(); // Clear camera list for this test

        visionSubsystem.periodic();

        assertNull(visionSubsystem.getBestTag());
        assertNull(visionSubsystem.getBestTag(1, 2, 3, 4));
    }

    @Test
    void testGetBestTag_WithSpecificFiducialIDs() {
        // Arrange
        VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);
        visionSubsystem.cameraList.clear(); // Clear camera list for this test

        PhotonVisionCamera mockCamera = mock(PhotonVisionCamera.class);
        visionSubsystem.cameraList.add(mockCamera);

        AprilTag mockTag = new AprilTag(1, new Pose3d());
        when(mockCamera.getBestTag(1, 2, 3)).thenReturn(mockTag);

        // Act
        AprilTag result = visionSubsystem.getBestTag(1, 2, 3);

        // Assert
        assertNotNull(result);
        assertEquals(1, result.ID);
    }

    @Test
    void testGetBestTag_WithNoMatchingFiducialIDs() {
        // Arrange
        VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);
        visionSubsystem.cameraList.clear(); // Clear camera list for this test

        PhotonVisionCamera mockCamera = mock(PhotonVisionCamera.class);
        visionSubsystem.cameraList.add(mockCamera);

        when(mockCamera.getBestTag(4, 5, 6)).thenReturn(null);

        // Act
        AprilTag result = visionSubsystem.getBestTag(4, 5, 6);

        // Assert
        assertNull(result);
    }

    @Test
    void testGetBestTag_WithNoFiducialIDs() {
        // Arrange
        VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);
        visionSubsystem.cameraList.clear(); // Clear camera list for this test

        PhotonVisionCamera mockCamera = mock(PhotonVisionCamera.class);
        visionSubsystem.cameraList.add(mockCamera);

        AprilTag mockTag = new AprilTag(2, new Pose3d());
        when(mockCamera.getBestTag()).thenReturn(mockTag);

        // Act
        AprilTag result = visionSubsystem.getBestTag();

        // Assert
        assertNotNull(result);
        assertEquals(2, result.ID);
    }
}
