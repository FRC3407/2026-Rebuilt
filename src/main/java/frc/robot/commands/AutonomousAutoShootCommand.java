// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousAutoShootCommand extends AnotherAutoShootCommand {
    public AutonomousAutoShootCommand(ShooterSubsystem m_shooter, DriveSubsystem m_drive) {
        super(m_shooter, m_drive);
        SmartDashboard.putData("Autonomous Shoot Command", this);
    } 
    @Override
    public boolean isFinished() {
        if(super.getShotCounter() > 6){
          return true;
        }
        return super.isFinished();
    }
}
