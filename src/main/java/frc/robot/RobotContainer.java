// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    public final DriveSubsystem m_robotDrive;
    public final VisionSubsystem m_vision;

    // The driver's controllers
    private final CommandJoystick leftJoystick = new CommandJoystick(OIConstants.kLeftJoystickPort);
    private final CommandJoystick rightJoystick = new CommandJoystick(OIConstants.kRightJoystickPort);
    private final CommandXboxController xboxController = new CommandXboxController(OIConstants.kXboxControllerPort);

    // Dashboard chooser for autonomous command
    private final SendableChooser<Command> autoChooser;

    private static RobotContainer instance;

    public static synchronized RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    private RobotContainer() {
        m_robotDrive = new DriveSubsystem();
        m_vision = new VisionSubsystem(m_robotDrive);

        configureButtonBindings();

        autoChooser = configureAutonomous();

        configureDashboard();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or
     * one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        // The right stick controls translation of the robot.
        // Turning is controlled by the X axis of the left stick.
        m_robotDrive.setDefaultCommand(new DriveCommand(
                rightJoystick::getY,
                rightJoystick::getX,
                leftJoystick::getX,
                m_robotDrive));

        // Button 7 on the right stick resets the gyro
        rightJoystick.button(7).onTrue(
                new InstantCommand(m_robotDrive::zeroHeading));

        xboxController.back().whileTrue(new RunCommand(
                m_robotDrive::setSwerveModulesToX,
                m_robotDrive));
    }

    /**
     * Use this method to set up all autonomous routines and add them to a
     * dashboard chooser. For PathPlanner, use this method to register
     * NamedCommands and EventTriggers.
     *
     * @return the autonomous chooser.
     */
    private SendableChooser<Command> configureAutonomous() {
        // TODO: register NamedCommands here
        SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
        // TODO: configure additional autonomous routines here
        return chooser;
    }

    /**
     * Use this method for additional configuration to the driver's dashboard.
     * SmartDashboard data should be added from subsystems and commands, but
     * this method defines the dashboard widgets.
     */
    private void configureDashboard() {
        SmartDashboard.putData("Auto Chooser", autoChooser);

        m_robotDrive.odometryDisplay.setRobotPose(new Pose2d());

        m_robotDrive.gyroDisplay.setNumber(0.0);

        m_vision.apriltagsVisibleDisplay.setBoolean(false);
    }

    /**
     * Use this to return the autonomous command to the main {@link Robot}
     * class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
