// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShooterConfig;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private SparkMax m_spindexer = new SparkMax(ShooterConstants.kSpindexerCanId,MotorType.kBrushless);
    private SparkFlex m_shooter = new SparkFlex(ShooterConstants.kShooterCanId,MotorType.kBrushless);

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        m_spindexer.configure(ShooterConfig.kSpindexerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        m_shooter.configure(ShooterConfig.kShooterConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /** @param speed speed from -1 to 1 */
    public void setShooterSpeed(double speed){
        m_shooter.set(speed);
    }

    /** @param speed speed from -1 to 1 */
    public void setSpindexerSpeed(double speed){
        m_spindexer.set(speed); 
    }
}
