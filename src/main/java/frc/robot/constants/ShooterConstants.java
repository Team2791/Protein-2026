package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the shooter subsystem, including motor configuration and
 * the distance-to-velocity regression used for automatic speed control.
 */
public final class ShooterConstants {

    private ShooterConstants() {}

    /** Motor configuration constants for the shooter flywheel motors. */
    public static final class Motor {

        private Motor() {}

        /** Whether the follower motor should spin opposite to the leader. */
        public static final boolean kInvertFollower = true;

        /** Idle mode for the shooter motors (coast allows wheels to spin freely when stopped). */
        public static final IdleMode kIdleMode = IdleMode.kCoast;
    }

    /** Manual shooter speed setpoints. */
    public enum Setpoint {
        /** Automatic (regression-powered) control */
        kRegress(0),
        /** Idle Mode */
        kLow(-50),
        /** Close-range shot. */
        kNear(-235),
        /** Mid-range shot. */
        kMedium(-285),
        /** Far-range shot. */
        kFar(-315);

        /** Flywheel velocity in rad/s. */
        public final double velocity;

        Setpoint(double velocity) {
            this.velocity = velocity;
        }
    }

    /**
     * Distance-to-velocity regression coefficients for automatic shooter speed control.
     *
     * <p>
     * The regression formula is: {@code velocity = kSq * dist² + kLin * dist + kConst}
     * where {@code dist} is the distance from the robot to the hub in meters.
     *
     * <p>
     * All coefficients are zero until tuned for the specific robot/game setup.
     */
    public static final class Regression {

        private Regression() {}

        /** Linear coefficient for the distance regression. */
        public static final double kLin = 39.964;

        /** Constant (offset) for the distance regression. */
        public static final double kConst = 168.311;

        /**
         * Applies the regression to compute the required shooter velocity at a given distance.
         *
         * @param dist Distance from robot to hub in meters
         * @return The required flywheel velocity in radians/second
         */
        public static double apply(double dist) {
            return kLin * dist + kConst;
        }
    }

    /**
     * X-position threshold (blue-alliance frame) beyond which the shooter spins down.
     *
     * <p>
     * When the robot's blue-frame X position exceeds this value, the robot is too
     * far from the hub to shoot and the flywheel is commanded to stop.
     */
    public static final double kSpinUpThreshold = Inches.of(215.61).in(Meters);

    /** Constants for the {@link frc.robot.commands.shooter.Shoot} command. */
    public static final class Shoot {

        private Shoot() {}

        /** Minimum time (seconds) the spindexer runs before checking for a stall. */
        public static final double kStallTimeout = 1.0;

        /** Velocity threshold (rad/s) below which the spindexer is considered stalled. */
        public static final double kStallVelocity = 200;

        /** Duration (seconds) the spindexer reverses between shots. */
        public static final double kReverseDuration = 0.25;
    }

    public static final Transform2d kBotToShooter = new Transform2d(
        Units.inchesToMeters(-8.0625),
        Units.inchesToMeters(8.25),
        new Rotation2d()
    );

    public static final double kAimFactor = 1.0;
    public static final double kWheelRadius = Units.inchesToMeters(2.0);
}
