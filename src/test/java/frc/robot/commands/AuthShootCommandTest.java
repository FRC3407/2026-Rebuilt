package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

import static frc.robot.commands.AutoShootCommand.getShooterSpeed;

/**
 * Add your docs here.
 */
public class AuthShootCommandTest {

    /**
     * Acceptable error.
     */
    final double DELTA = 0.05;

    @Test
    void testGetShooterSpeed_exact_match_to_data() {
        assertEquals(0.725, getShooterSpeed(2.7), DELTA);
        assertEquals(0.77, getShooterSpeed(3.7), DELTA);
        assertEquals(0.62, getShooterSpeed(1.8), DELTA);
        assertEquals(0.70, getShooterSpeed(2.3), DELTA);
        assertEquals(0.815, getShooterSpeed(3.3), DELTA);
    }

    @Test
    void testGetShooterSpeed_interpolating() {
        assertEquals(0.67, getShooterSpeed(2.25), DELTA, "Interpolating between 1.8 and 2.7 meters");
        assertEquals(0.79, getShooterSpeed(3.5), DELTA, "Interpolating between 3.3 and 3.7 meters");
        assertEquals(0.71, getShooterSpeed(2.5), DELTA, "Interpolating between 2.3 and 2.7 meters");
        assertEquals(0.80, getShooterSpeed(3.4), DELTA, "Interpolating between 3.3 and 3.7 meters");
    }
}
