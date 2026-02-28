// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeeperSubsystem extends SubsystemBase {

    private final DigitalOutput output = new DigitalOutput(3);
    private double secondsRemaining;

    @Override
    public void periodic() {
        secondsRemaining -= 0.02;
        output.set(secondsRemaining > 0);
        // This method will be called once per scheduler run
    }

    public void beep(double seconds) {
        secondsRemaining = seconds;
    }
}
