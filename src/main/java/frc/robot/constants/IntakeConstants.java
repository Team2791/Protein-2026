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

        public enum RollerState {
            kNormal(-0.5),
            kStopped(0),
            kReverse(-RollerState.kNormal.power);

            public final double power;

            RollerState(double power) {
                this.power = power;
            }
        }
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
        public static final double kReduction = 9;

        public static final double kVelocityThreshold = 1;

        /** Position delta (rad) above the deployed rest position that triggers corrective downward power. */
        public static final double kPushThreshold = 0.1;

        /** Duty cycle applied to the pivot motor during deploy/retract. */
        public static final double kDeployPower = 0.3;
    }
}
