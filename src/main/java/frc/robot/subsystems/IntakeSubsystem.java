// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfig;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private static final double deploySetPoint = 0.0; // TODO: LIKE REALLY TODO: set a real value before using
    private static final double deploySpeed = 0.1; // TODO: LIKE REALLY TODO: set a real value before using

    private final SparkMax m_intakeMotor = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    private final SparkMax m_deployMotor = new SparkMax(IntakeConstants.kDeployLeftCanId, MotorType.kBrushless);
    private RelativeEncoder m_Encoder = m_deployMotor.getEncoder();
    private boolean isDeploying = false;
    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        m_intakeMotor.configure(IntakeConfig.kIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_deployMotor.configure(IntakeConfig.kDeployLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_Encoder.setPosition(0);
        isDeploying = false;
        // m_leftEncoder.setPosition(0);
        // m_rightEncoder.setPosition(0);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if(isDeploying){
            // run at `deploySpeed` if its not there yet. if it is, then stop.
            m_deployMotor.set(m_Encoder.getPosition() < deploySetPoint ? deploySpeed : 0);
            if (m_Encoder.getPosition() >= deploySetPoint)
                isDeploying = false;
            // ballerrrrr
        } else {
            m_deployMotor.set(0);
        }

    }

    public void setIntakeSpeed(double speed) {
        m_intakeMotor.set(speed);
    }


    /** Deploys the intake out */
    public void startDeploy(){
        isDeploying = true;
    }
    public void stopDeploy(){
        m_deployMotor.set(0);
        isDeploying = false;
    }
}
