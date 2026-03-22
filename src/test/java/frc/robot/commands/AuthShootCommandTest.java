package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static frc.robot.commands.AutoShootCommand.getShooterSpeed;

/**
 * Add your docs here.
 */
public class AuthShootCommandTest {

    @Test
    void testGetShooterSpeed() {
        assertEquals(0.65, getShooterSpeed(inchesToMeters(42)));
        assertEquals(0.75, getShooterSpeed(inchesToMeters(51)));
        assertEquals(0.75, getShooterSpeed(inchesToMeters(54)));
        assertEquals(0.95, getShooterSpeed(inchesToMeters(81)));
        assertEquals(0.95, getShooterSpeed(inchesToMeters(91)));

        assertEquals(0.72, getShooterSpeed(inchesToMeters(48)), 0.01, "Interpolating between 42 and 54 inches");
        assertEquals(0.95, getShooterSpeed(inchesToMeters(86)), 0.01, "Interpolating between 81 and 91 inches");

        assertEquals(0.75, getShooterSpeed(inchesToMeters(60)), 0.01, "Interpolating");
        assertEquals(0.82, getShooterSpeed(inchesToMeters(70)), 0.01, "Interpolating");
        assertEquals(1.01, getShooterSpeed(inchesToMeters(100)), 0.01, "Interpolating");
    }
}
