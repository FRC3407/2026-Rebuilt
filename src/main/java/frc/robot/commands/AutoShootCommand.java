package frc.robot.commands;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* Shoot using shooter speeds based on distance from Hub. */
public class AutoShootCommand extends Command {

    private final ShooterSubsystem m_shooter;
    private final DriveSubsystem m_robotDrive;
    private final Timer timer = new Timer();

    /**
     * Delay time in seconds.
     */
    public final double SPINDEXER_TIMEOUT = 0.5;

    public AutoShootCommand(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {
        m_shooter = shooterSubsystem;
        m_robotDrive = driveSubsystem;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        double distanceToHub = m_robotDrive.distanceToHub();
        m_shooter.setShooterSpeed(getShooterSpeed(distanceToHub));
        m_shooter.setSpindexerSpeed(timer.hasElapsed(SPINDEXER_TIMEOUT) ? 1.0 : 0.0);

    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setShooterSpeed(0);
        m_shooter.setSpindexerSpeed(0);
    }

    private static Map<Double, Double> shooterData = new TreeMap<>();

    static {
        shooterData.put(2.7, 0.725);
        shooterData.put(3.7, 0.77);
        shooterData.put(1.8, 0.62);
        shooterData.put(2.3, 0.7);
        shooterData.put(3.3, 0.815);

        shooterData.put(0.0, 0.62);
        shooterData.put(100.0, 1.00);
    }

    /**
     * Calculate shooter speed based on previosly recorded shooter data.
     *
     * @param distanceToHub Distance in meters.
     * @return Shooter speed from in RPM.
     */
    protected static double getShooterSpeed(double distanceToHub) {
        double d1 = 0.0, d2 = 1000.0, p1 = 0.0, p2 = 10.0, r = 0.0;
        d1 = shooterData.keySet().iterator().next();
        p1 = shooterData.get(d1);

        for (double d : shooterData.keySet()) {
            if (d > distanceToHub) {
                d2 = d;
                p2 = shooterData.get(d);
                break;
            }
            d1 = d;
            p1 = shooterData.get(d);
        }

        r = (distanceToHub - d1) / (d2 - d1);

        return p1 + r * (p2 - p1);
    }

    /**
     * Simplified shooter speed calculation using only a linear function.
     *
     * @param distanceToHub Distance in meters.
     * @return Shooter speed from in RPM.
     */
    protected static double getShooterSpeed_simple(double distanceToHub) {
        return 0.0879 * distanceToHub + 0.483;
    }
}
