package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Constants for the climber subsystem.
 *
 * <p>
 * Contains motor configuration, gear reduction, and position setpoints
 * for the axle rotation mechanism. Position values are in radians at the
 * output shaft (direct drive, reduction = 1.0).
 */
public final class ClimberConstants {

    private ClimberConstants() {}

    /** Motor configuration constants for the climber axle motors. */
    public static final class Motor {

        private Motor() {}

        /** Idle mode for axle motors (brake holds position). */
        public static final IdleMode kIdleMode = IdleMode.kBrake;

        /** Whether the follower motor should spin opposite to the leader. */
        public static final boolean kInvertFollower = false;
    }

    /**
     * Gear reduction from motor shaft to axle output.
     *
     * <p>
     * Convention: N motor turns : 1 output turn.
     */
    public static final double kReduction = 1.0;

    /**
     * Axle position setpoints.
     *
     * <p>
     * Each variant carries a radian value at the output shaft.
     */
    public enum Position {
        /** Fully contracted (home position). */
        CONTRACT(0.0),
        /** Fully expanded. */
        EXPAND(3.0),
        /** Partial contraction for autonomous L1 (allows descent). */
        CONTRACT_PARTIAL(1.0);

        /** Position setpoint in radians. */
        public final double radians;

        Position(double radians) {
            this.radians = radians;
        }
    }

    /**
     * Duration in seconds to wait after actuating the pneumatic cylinders.
     *
     * <p>
     * Allows the solenoids to fully extend or retract before the command ends.
     */
    public static final double kEngageDuration = 0.5;

    /** Duration in seconds to reverse drivetrain after descent */
    public static final double kReverseDuration = 0.5;

    /** Speed of drivetrain when reversing */
    public static final double kReverseSpeed = 0.5;
}
