package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/**
 * Subsystem to controll all the lights running on the external lights
 * microcontroller.
 * <p>
 * The whole "lights plan" should be implemented from within this subsystem,
 * controlled mostly from within the {@code periodic()} method.
 * <p>
 * If you need external subsystems and commands to communicate into this class,
 * create "modes" within the class to indicate external states. For instance,
 * if you need a command to indicate when the robot is preparing to shoot, add
 * a {@code setShootingMode(boolean)} to this class. Then, add code to the
 * {@code periodic()} that changes animations based on current modes.
 */
public class LightsSubsystem extends SubsystemBase {

    public static final int MAX_ANIMATIONS = 1; // Must be 32 or less
    public static final int MAX_STRIPS = 4; // Must be 8 or less
    public static int I2C_ADDRESS = 0x41;
    private static int MAX_PARAM = 64;

    // Strips:
    public static final int LEFT_EYE = 0;
    public static final int RIGHT_EYE = 1;
    public static final int LEFT_PANEL = 2;
    public static final int RIGHT_PANEL = 3;

    // Animations:
    public static final int LADDER_RED = 0;
    public static final int TEAM_NUMBER = 1;
    public static final int SHOOTING = 2;
    public static final int EYES_ANIM = 17;

    private AnimationState[] currentAnimation = new AnimationState[MAX_STRIPS];
    private AnimationState[] nextAnimation = new AnimationState[MAX_STRIPS];
    private byte[] dataOut = new byte[MAX_PARAM];

    private I2C i2c = null;

    private final VisionSubsystem m_vision;
    private final ShooterSubsystem m_shooter;
    private final DriveSubsystem m_drive;

    public LightsSubsystem(VisionSubsystem vision, ShooterSubsystem shooter, DriveSubsystem drive) {
        this.m_vision = vision;
        this.m_shooter = shooter;
        this.m_drive = drive;
        i2c = new I2C(Port.kOnboard, I2C_ADDRESS);
        for (int s = 0; s < MAX_STRIPS; s++) {
            currentAnimation[s] = new AnimationState(0, null);
            nextAnimation[s] = new AnimationState(MAX_ANIMATIONS, null);
        }
        clearAllAnimations();
    }

    @Override
    public void periodic() {
        // TODO: monitor internal robot state and change animations as necessary
        // using setAnimation or clearAllAnimations

        double distanceToHub = m_drive.distanceToHub();

        if (DriverStation.isDisabled() || DriverStation.isTest()) {
            setAnimation(LEFT_PANEL, TEAM_NUMBER);
            setAnimation(RIGHT_PANEL, TEAM_NUMBER);
            setAnimation(LEFT_EYE, EYES_ANIM);
            setAnimation(RIGHT_EYE, EYES_ANIM);

        } else if (DriverStation.isAutonomous()) {
            setAnimation(LEFT_EYE, EYES_ANIM);
            setAnimation(RIGHT_EYE, EYES_ANIM);
            if (m_shooter.getShooterSpeed() != 0.0) {
                setAnimation(LEFT_PANEL, SHOOTING);
                setAnimation(RIGHT_PANEL, SHOOTING);
            } else {
                setAnimation(LEFT_PANEL, TEAM_NUMBER);
                setAnimation(RIGHT_PANEL, TEAM_NUMBER);
            }

        } else if (DriverStation.isTeleop()) {
            setAnimation(LEFT_EYE, EYES_ANIM);
            setAnimation(RIGHT_EYE, EYES_ANIM);
            if (distanceToHub >= 2.7 && distanceToHub <= 3.3) {
                setAnimation(LEFT_PANEL, LADDER_RED);
                setAnimation(RIGHT_PANEL, LADDER_RED);
            } else if (m_shooter.getShooterSpeed() != 0.0) {
                setAnimation(LEFT_PANEL, SHOOTING);
                setAnimation(RIGHT_PANEL, SHOOTING);
            } else {
                setAnimation(LEFT_PANEL, TEAM_NUMBER);
                setAnimation(RIGHT_PANEL, TEAM_NUMBER);
            }
        }

        sendAllAnimations();
    }

    /**
     * Clear out all the strips and stop all animations.
     * <br>
     * This should not be called from outside this subsystem.
     */
    protected void clearAllAnimations() {
        for (int s = 0; s < MAX_STRIPS; s++) {
            nextAnimation[s].animNumber = MAX_ANIMATIONS;
            nextAnimation[s].param = null;
        }
    }

    /**
     * Set one strip to have the numbered animation.
     * <br>
     * This should not be called from outside this subsystem. It should only be
     * called from within the "Lights Plan" implemented within the {@code periodic}
     * method.
     */
    protected void setAnimation(int stripNumber, int animNumber) {
        nextAnimation[stripNumber].animNumber = animNumber;
        nextAnimation[stripNumber].param = null;
    }

    /**
     * Push out all animation changes to the Lights Board. <br/>
     * This program takes a <em>lazy</em> approach, in that animation signals are
     * only sent out if they <em>need</em> to change. Signals are only sent if the
     * desired animation is different from the current animation.
     * This prevents redundant, unnecessary changes from dominating the I2C bus.
     */
    private void sendAllAnimations() {
        for (int s = 0; s < MAX_STRIPS; s++) {
            if (!nextAnimation[s].equals(currentAnimation[s])) {
                sendOneAnimation(s);
                currentAnimation[s].animNumber = nextAnimation[s].animNumber;
                currentAnimation[s].param = nextAnimation[s].param;
            }
        }
    }

    /**
     * Send out one message on I2C to change the animation on one strip to be one
     * specific animation. The {@code stripNumber} and {@code animNumber} are packed
     * into a single byte. If there is a {@code param} associated with this change,
     * it is concatenated onto the message.
     */
    private void sendOneAnimation(int stripNumber) {
        int dataLength = 0;
        int animNumber = nextAnimation[stripNumber].animNumber;
        Integer b = Integer.valueOf(((stripNumber << 5) & 0xE0) | (animNumber & 0x1F));
        dataOut[dataLength++] = b.byteValue();

        if (nextAnimation[stripNumber].param != null && nextAnimation[stripNumber].param.length() > 0) {
            for (int i = 0; i < nextAnimation[stripNumber].param.length() && i < MAX_PARAM; i++) {
                dataOut[dataLength++] = nextAnimation[stripNumber].param.getBytes()[i];
            }
        }

        i2c.writeBulk(dataOut, dataLength);
    }

    private static class AnimationState {
        public AnimationState(int a, String p) {
            this.animNumber = a;
            this.param = p;
        }

        public int animNumber;
        public String param;

        @Override
        public boolean equals(Object obj) {
            if (obj == null || !(obj instanceof AnimationState)) {
                return false;
            }
            AnimationState other = (AnimationState) obj;
            if (this.animNumber != other.animNumber) {
                return false;
            }
            if (this.param == null) {
                return other.param == null;
            } else {
                return this.param.equals(other.param);
            }
        }
    }
}