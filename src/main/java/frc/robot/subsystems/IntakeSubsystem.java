// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfig;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    SparkMax m_intakeMotor = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    SparkMax m_deployLeftMotor = new SparkMax(IntakeConstants.kDeployLeftCanId, MotorType.kBrushless);
    SparkMax m_deployRightMotor = new SparkMax(IntakeConstants.kDeployRightCanId, MotorType.kBrushless);

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        m_intakeMotor.configure(IntakeConfig.kIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_deployLeftMotor.configure(IntakeConfig.kDeployLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_deployRightMotor.configure(IntakeConfig.kDeployRightConfig.follow(m_deployLeftMotor),
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setIntakeSpeed(double speed) {
        m_intakeMotor.set(speed);
    }


    /** Deploys the intake out */
    public void deployOut() {
        // m_deployLeftMotor.
    }

    /** Retracts the intake back */
    public void deployIn() {

    }
}
