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

    private final SparkMax m_intakeMotor = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    private final SparkMax m_deployLeftMotor = new SparkMax(IntakeConstants.kDeployLeftCanId, MotorType.kBrushless);
    private final SparkMax m_deployRightMotor = new SparkMax(IntakeConstants.kDeployRightCanId, MotorType.kBrushless);
    private final PIDController m_control = new PIDController(1, 0, 0); // find good pid values
    private final double out_position = 0.0; //read encoders to get proper value
    private final double in_position = 0.0;  //same
    private RelativeEncoder m_leftEncoder = m_deployLeftMotor.getEncoder();
    private RelativeEncoder m_rightEncoder = m_deployRightMotor.getEncoder();
    private double set_point;
    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        m_intakeMotor.configure(IntakeConfig.kIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_deployLeftMotor.configure(IntakeConfig.kDeployLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_deployRightMotor.configure(IntakeConfig.kDeployRightConfig.follow(m_deployLeftMotor),
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_leftEncoder = m_deployLeftMotor.getEncoder();
        m_rightEncoder = m_deployRightMotor.getEncoder();
        // m_leftEncoder.setPosition(0);
        // m_rightEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double lmotor_speed = MathUtil.clamp(-0.1 + m_control.calculate(m_leftEncoder.getPosition(), set_point), -1.0,1.0);
        double rmotor_speed = MathUtil.clamp(-0.1 + m_control.calculate(m_rightEncoder.getPosition(), set_point), -1.0,1.0);
        m_deployLeftMotor.set(lmotor_speed);
        m_deployRightMotor.set(rmotor_speed);

    }

    public void setIntakeSpeed(double speed) {
        m_intakeMotor.set(speed);
    }


    /** Deploys the intake out */
    public void deployOut() {
        set_point = out_position;
        m_control.reset();
    }

    /** Retracts the intake back */
    public void deployIn() {
        set_point = in_position;
        m_control.reset();
    }
}
