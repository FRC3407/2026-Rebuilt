package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static frc.robot.commands.AutoShootCommand.getShooterSpeed;

/**
 * Add your docs here.
 */
public class AuthShootCommandTest {

    /** Acceptable error. */
    final double DELTA = 0.05;

    @Test
    void testGetShooterSpeed() {
        assertEquals(0.65, getShooterSpeed(inchesToMeters(42)), DELTA);
        assertEquals(0.75, getShooterSpeed(inchesToMeters(51)), DELTA);
        assertEquals(0.75, getShooterSpeed(inchesToMeters(54)), DELTA);
        assertEquals(0.95, getShooterSpeed(inchesToMeters(81)), DELTA);
        assertEquals(0.95, getShooterSpeed(inchesToMeters(91)), DELTA);

        assertEquals(0.72, getShooterSpeed(inchesToMeters(48)), DELTA, "Interpolating between 42 and 54 inches");
        assertEquals(0.95, getShooterSpeed(inchesToMeters(86)), DELTA, "Interpolating between 81 and 91 inches");

        assertEquals(0.75, getShooterSpeed(inchesToMeters(60)), DELTA, "Interpolating");
        assertEquals(0.82, getShooterSpeed(inchesToMeters(70)), DELTA, "Interpolating");
        assertEquals(1.01, getShooterSpeed(inchesToMeters(100)), DELTA, "Interpolating");
    }
}
