// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Newton;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfig;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax m_intakeMotor = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    private final SparkMax m_deployMotor = new SparkMax(IntakeConstants.kDeployLeftCanId, MotorType.kBrushless);
    private SparkLimitSwitch m_toplimitSwitch = m_deployMotor.getForwardLimitSwitch();
    private SparkLimitSwitch m_bottomlimitSwitch = m_deployMotor.getReverseLimitSwitch();
    private final PIDController m_control = new PIDController(1, 0, 0); // TODO: find good pid values
    private RelativeEncoder m_Encoder = m_deployMotor.getEncoder();
    private double set_point = 0; // TODO: use encoders and find out correct value later
    private boolean isDeploying;

    private double minError = 1;
    private double deploy_offset = 0;
    private Timer timer = new Timer();

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        m_intakeMotor.configure(IntakeConfig.kIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_deployMotor.configure(IntakeConfig.kDeployLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_Encoder.setPosition(0);
        isDeploying = false;
    }

    @Override
    public void periodic() {
        double maxSpeed = 0.3;
        if (m_toplimitSwitch.isPressed()){ //Not sure if top or bottom is actually top or bottom. Switch these if it doesn't work
            m_Encoder.setPosition(0);
        }
        if (m_bottomlimitSwitch.isPressed()){
            m_Encoder.setPosition(deployAngle);
            // deploy_offset = IntakeConstants.deployAngle - m_Encoder.getPosition(); why
        }
        if (timer.hasElapsed(4.0)) {
            stopDeploy();
        }
        if (isDeploying) {
            double motor_speed = MathUtil.clamp(m_control.calculate(m_Encoder.getPosition() , set_point), -maxSpeed, maxSpeed);
            m_deployMotor.set(motor_speed);
            if (Math.abs(m_Encoder.getPosition() - set_point) < minError) {
                stopDeploy();
            }
        }
        SmartDashboard.putNumber("deploy positon", m_Encoder.getPosition());
    }

    public void setIntakeSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    public void setSetPoint(double point) {
        System.out.println("setpoint: "+set_point+" -> "+point);
        set_point = point;
        startDeploy();
    }

    public void setDeploySpeed(double speed) {
        m_deployMotor.set(0.3);
    }

    /** Deploys the intake out */
    public void startDeploy() {
        timer.restart();
        isDeploying = true;
    }

    public void stopDeploy() {
        m_deployMotor.set(0);
        isDeploying = false;
    }
}
