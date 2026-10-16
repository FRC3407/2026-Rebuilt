// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShooterConfig;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private SparkMax m_spindexer = new SparkMax(ShooterConstants.kSpindexerCanId, MotorType.kBrushless);
    private SparkFlex m_shooter = new SparkFlex(ShooterConstants.kShooterCanId, MotorType.kBrushless);
    private SparkMax m_agitator = new SparkMax(ShooterConstants.kAgitatorCanId, MotorType.kBrushed);

    private RelativeEncoder m_shooterEncoder = m_shooter.getEncoder();

    public double scalingFactor = 5500;
    private double targetSpeed;

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        m_spindexer.configure(ShooterConfig.kSpindexerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        m_shooter.configure(ShooterConfig.kShooterConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        m_agitator.configure(ShooterConfig.kShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        SmartDashboard.putData(this);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("motor speed", m_shooterEncoder::getVelocity, null);
        builder.addDoubleProperty("target speed", () -> targetSpeed, null);
        builder.addDoubleProperty("rpm scaling factor", () -> scalingFactor, null);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("ShooterSubsystem/ShooterSpeed", getShooterSpeed());
    }

    /** @param speed speed from -1 to 1 */
    public void setShooterSpeed(double speed) {
        // m_shooter.set(speed);
        setShooterSpeedRpm(speed * scalingFactor);
    }

    public void setShooterSpeedRpm(double rpm) {
        targetSpeed = rpm;
        if (rpm == 0) {
            m_shooter.set(0);
        } else {
            m_shooter.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
        }
    }

    /** @param speed speed from -1 to 1 */
    public void setSpindexerSpeed(double speed) {
        m_spindexer.set(-speed); // spindexer runs backwards when speed is +
    }
    public void setAgitatorSpeed(double speed) {
        m_agitator.set(-speed);
    }
    public double getShooterSpeed() {
        return m_shooterEncoder.getVelocity();
    }
    public double getSpindexerSpeed() {
        return m_spindexer.get();
    }
}
