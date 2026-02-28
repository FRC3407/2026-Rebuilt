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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfig;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax m_intakeMotor = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    private final SparkMax m_deployMotor = new SparkMax(IntakeConstants.kDeployLeftCanId, MotorType.kBrushless);
    private final PIDController m_control = new PIDController(1, 0, 0); // TODO: find good pid values
    private RelativeEncoder m_Encoder = m_deployMotor.getEncoder();
    private double set_point = 0; // TODO: use encoders and find out correct value later
    private boolean isDeploying;
    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        m_intakeMotor.configure(IntakeConfig.kIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_deployMotor.configure(IntakeConfig.kDeployLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_Encoder.setPosition(0);
        isDeploying = false;
        // m_leftEncoder.setPosition(0);
        // m_rightEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(isDeploying){
            double motor_speed = MathUtil.clamp(-0.1 + m_control.calculate(m_Encoder.getPosition(), set_point), -1.0,1.0);
            m_deployMotor.set(motor_speed);
        }
        if (m_Encoder.getPosition() >= set_point){
            stopDeploy();
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
