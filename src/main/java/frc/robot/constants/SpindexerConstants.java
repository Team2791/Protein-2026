package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Constants for the spindexer subsystem.
 *
 * <p>Contains motor idle mode and fixed duty-cycle powers for each motor.
 * Adjust {@link #kSpindexerPower} and {@link #kKickerPower} during tuning.
 */
public final class SpindexerConstants {

    private SpindexerConstants() {}

    /** Motor configuration constants for the spindexer motors. */
    public static final class Motor {

        private Motor() {}

        /** Idle mode for the spindexer motors (brake holds position when stopped). */
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }

    /**
     * Duty-cycle power for the SparkFlex (Neo Vortex) motor, in the range [-1, 1].
     *
     * <p>Positive values spin the motor forward. Tune this value on the robot.
     */
    public static final double kSpindexerPower = 0.5;

    /**
     * Duty-cycle power for the SparkMax (Neo) motor, in the range [-1, 1].
     *
     * <p>Positive values spin the motor forward. Tune this value on the robot.
     */
    public static final double kKickerPower = 0.5;
}
