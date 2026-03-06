package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
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
        Pose2d expectedPose;
        if(Robot.isSimulation()) {
            expectedPose = new Pose2d(2, 2, Rotation2d.fromDegrees(90));
        } else {
            expectedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        }
        assertNotNull(pose2d);
        assertEquals(pose2d.getTranslation().getX(), expectedPose.getX(), "Initial X position should be 0.0");
        assertEquals(pose2d.getTranslation().getY(), expectedPose.getY(), "Initial Y position should be 0.0");
        assertEquals(pose2d.getRotation().getDegrees(), expectedPose.getRotation().getDegrees(), "Initial rotation should be 0.0 degrees");
    }
}
