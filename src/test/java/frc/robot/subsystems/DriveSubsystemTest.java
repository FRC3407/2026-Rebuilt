package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;

public class DriveSubsystemTest {

    static RobotContainer robotContainer;
    static DriveSubsystem driveSubsystem;

    @BeforeAll
    static void setup() {
        robotContainer = RobotContainer.getInstance();
        driveSubsystem = robotContainer.m_robotDrive;
    }

    @Test
    void testInitialization() {
        assertNotNull(driveSubsystem, "DriveSubsystem should be initialized properly.");
    }

    @Test
    void testGetPose() {
        Pose2d pose2d = driveSubsystem.getPose();
        assertNotNull(pose2d);
        assertEquals(pose2d.getTranslation().getX(), 0.0, "Initial X position should be 0.0");
        assertEquals(pose2d.getTranslation().getY(), 0.0, "Initial Y position should be 0.0");
        assertEquals(pose2d.getRotation().getDegrees(), 0.0, "Initial rotation should be 0.0 degrees");
    }
}
