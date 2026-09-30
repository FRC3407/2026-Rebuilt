import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.TargetCommand;
import frc.robot.subsystems.DriveSubsystem;


public class TargetCommandTest{

    static RobotContainer robotContainer;
    static DriveSubsystem driveSubsystem;
    double yoffset = 0.2286;
    double xoffset = 0.32385;
    @BeforeAll
    static void setup() {
        robotContainer = RobotContainer.getInstance();
        driveSubsystem = robotContainer.m_robotDrive;
        /*facing in + y 0.2286m  y
                        0.32385m x}
        */

    }
    @Test
    void testInitialization() {
        assertNotNull(driveSubsystem, "DriveSubsystem should be initialized properly.");
    }

    @Test
    void testShooterTransform() {
        Pose2d startPose1 = new Pose2d(0, 4.035, new Rotation2d());
        Pose2d startPose2 = new Pose2d(0, 4.035, new Rotation2d(Math.PI/4));
        //Translates relative to rotation, but the starting rotation, not ending one, so shooter offset is incorrect as of 9/29/26
        TargetCommand targetcommand1 = new TargetCommand(()-> 0, ()-> 0, driveSubsystem);
        driveSubsystem.resetOdometry(startPose1);
        System.out.println(targetcommand1.getShooterTransformed());
        driveSubsystem.resetOdometry(startPose2);
        System.out.println(targetcommand1.getShooterTransformed());
    }
}
