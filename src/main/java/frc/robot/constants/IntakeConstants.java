package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the intake subsystem.
 *
 * <p>
 * Contains motor configuration, pivot setpoints, intake wheel geometry,
 * and gear reduction for both the roller and pivot mechanisms.
 */
public final class IntakeConstants {

    private IntakeConstants() {}

    /** Motor configuration constants for the intake roller motors. */
    public static final class Roller {

        private Roller() {}

        /** Idle mode for roller motors */
        public static final IdleMode kIdleMode = IdleMode.kBrake;

        /** Whether the follower motor should spin opposite to the leader. */
        public static final boolean kInvertFollower = true;

        /**
         * Gear reduction from motor shaft to roller wheel.
         *
         * <p>
         * Convention: N motor turn : 1 wheel turns
         * (i.e. a value {@literal >} 1 means the motor spins faster than the wheel).
         * Adjust this to match the physical gearbox ratio.
         */
        public static final double kReduction = 2;

        /**
         * Radius of the intake roller wheels in meters.
         *
         * <p>
         * Used to convert drivetrain linear speed (m/s) into wheel surface velocity
         * and then into motor angular velocity (rad/s).
         */
        public static final double kWheelRadius = Units.inchesToMeters(2);

        /** Target roller power */
        public static final double kPower = -0.3;
    }

    /** Motor configuration constants for the intake pivot motors. */
    public static final class Pivot {

        private Pivot() {}

        /** Idle mode for pivot motors (brake holds position). */
        public static final IdleMode kIdleMode = IdleMode.kBrake;

        /** Whether the follower motor should spin opposite to the leader. */
        public static final boolean kInvertFollower = true;

        /**
         * Gear reduction from motor shaft to pivot output.
         *
         * <p>
         * Convention: N motor turn : 1 output turn
         * (i.e. a value {@literal >} 1 means the motor spins faster than the output).
         * Adjust this to match the physical gearbox ratio.
         */
        public static final double kReduction = 20;

        /**
         * The deployed (down) position setpoint for the pivot, in radians.
         *
         * <p>
         * The retracted position is assumed to be 0 rad.
         */
        public static final double kDeployedPosition = -1.59;

        /**
         * Tolerance for considering the pivot "at position", in radians.
         *
         * <p>
         * Rollers will not spin until the pivot is within this threshold
         * of the deployed setpoint.
         */
        public static final double kTolerance = 0.11;
    }
}
