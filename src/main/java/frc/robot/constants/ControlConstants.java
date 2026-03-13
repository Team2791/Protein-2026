package frc.robot.constants;

import static frc.robot.util.MathPlus.kTau;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.TunableSparkPID;

/**
 * Control loop constants for robot motion control.
 *
 * <p>
 * Contains PID gains, feedforward coefficients, and control limits for:
 *
 * <ul>
 *   <li>Swerve module drive motors (velocity control)
 *   <li>Swerve module turn motors (position control)
 *   <li>Autonomous trajectory following
 *   <li>Vision-based alignment
 *   <li>Joystick input rate limiting
 * </ul>
 */
public final class ControlConstants {

    private ControlConstants() {}

    /**
     * Shooter mechanism PID constants.
     */
    public static final class Shooter {

        private Shooter() {}

        /** Tunable PID controller for shooter velocity control. */
        public static final TunableSparkPID kPid = new TunableSparkPID(
            0,
            0,
            0,
            "Shooter"
        );
    }

    /**
     * Intake mechanism PID constants.
     */
    public static final class Intake {

        private Intake() {}

        /** Tunable PID controller for roller velocity control. */
        public static final TunableSparkPID kRollerPid = new TunableSparkPID(
            0,
            0,
            0,
            "Intake/Roller"
        );

        /** Tunable PID controller for pivot position control. */
        public static final TunableSparkPID kPivotPid = new TunableSparkPID(
            0,
            0,
            0,
            "Intake/Pivot"
        );
    }

    /**
     * Climber mechanism PID constants.
     */
    public static final class Climber {

        private Climber() {}

        /** Tunable PID controller for axle position control. */
        public static final TunableSparkPID kPid = new TunableSparkPID(
            0,
            0,
            0,
            "Climber"
        );
    }

    /**
     * Autonomous trajectory following PID constants.
     *
     * <p>
     * Used by {@link frc.robot.autos.AutoManager} for Choreo trajectory following.
     */
    public static final class Auto {

        private Auto() {}

        /** Proportional gain for X and Y position errors (orthogonal axes). */
        public static final double kOrthoP = 1.25;
        /** Integral gain for orthogonal position errors. */
        public static final double kOrthoI = 0.00;
        /** Derivative gain for orthogonal position errors. */
        public static final double kOrthoD = 0.00;

        /** Proportional gain for rotation error. */
        public static final double kTurnP = 0.00;
        /** Integral gain for rotation error. */
        public static final double kTurnI = 0.00;
        /** Derivative gain for rotation error. */
        public static final double kTurnD = 0.00;

        /** Threshold to switch to nearby control. */
        public static final double kNearbyThreshold = 0.25;
    }

    /**
     * Vision-based alignment PID constants.
     *
     * <p>
     * Used by {@link frc.robot.commands.drive.pathfind.Nearby} for fine alignment
     */
    public static final class Nearby {

        private Nearby() {}

        /** Proportional gain for X and Y alignment errors. */
        public static final double kOrthoP = 4.75;
        /** Integral gain for alignment position errors. */
        public static final double kOrthoI = 0.00;
        /** Derivative gain for alignment position errors. */
        public static final double kOrthoD = 0.00;

        /** Proportional gain for alignment rotation error. */
        public static final double kTurnP = 2.60;
        /** Integral gain for alignment rotation error. */
        public static final double kTurnI = 0.00;
        /** Derivative gain for alignment rotation error. */
        public static final double kTurnD = 0.04;

        /** Maximum angular velocity during alignment (radians/second). TODO: May be increased. */
        public static final double kMaxTurnVelocity = kTau;
        /** Maximum angular acceleration during alignment (radians/second²). */
        public static final double kMaxTurnAcceleration = kTau;

        /** Position and rotation tolerance for "at target" detection. TODO: May need adjustment for shooter vs PnP. */
        public static final Pose2d kTolerance = new Pose2d(
            0.03,
            0.03,
            new Rotation2d(0.05)
        );
    }

    /** Drivetrain motion constraints. */
    public static final class Drivetrain {

        private Drivetrain() {}

        /** Velocity constraints for drivetrain motion. */
        public static final class MaxSpeed {

            private MaxSpeed() {}

            /** Maximum translational speed */
            public static final double kLinear =
                DriveConstants.maxSpeedMetersPerSec;

            /** Maximum angular speed */
            public static final double kAngular =
                DriveConstants.maxAngularSpeedRadPerSec;
        }
    }

    /**
     * Slew rate limiters for driver input smoothing.
     *
     * <p>
     * Limits acceleration of joystick commands to prevent abrupt movements
     * and reduce mechanical stress on the drivetrain.
     */
    public static final class SlewRateLimit {

        private SlewRateLimit() {}

        /** Orthogonal rate limiting in percent/second (1.0=100%) TODO: Can be increased (less top-heavy than previous design). */
        public static final double kOrthogonal = 1.667;
        /** Rotational rate limiting in percent/second (1.0=100%) */
        public static final double kRotational = 3.87;
    }
}
