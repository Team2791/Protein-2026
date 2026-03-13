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
     * Convention: 1 motor turn : {@code kReduction} output turns.
     * Currently direct drive (1:1).
     */
    public static final double kReduction = 1.0;

    /**
     * Contracted position setpoint in radians.
     *
     * <p>
     * This is the home/starting position of the axle.
     */
    public static final double kContract = 0.0;

    /**
     * Expanded position setpoint in radians.
     *
     * <p>
     * The axle rotates to this position to extend the climber mechanism.
     * Tune this value on the robot.
     */
    public static final double kExpand = 3.0;

    /**
     * Partial contraction position setpoint in radians.
     *
     * <p>
     * Used during autonomous to reach L1 in a position where the robot
     * can come back down. Between {@link #kContract} and {@link #kExpand}.
     */
    public static final double kContractPartial = 1.0;
}
