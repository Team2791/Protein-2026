package frc.robot.constants;

import frc.robot.subsystems.drive.DriveConstants;

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

        /** Proportional gain for shooter velocity control. */
        public static final double kP = 0.01;
        /** Integral gain for shooter velocity control. */
        public static final double kI = 0;
        /** Derivative gain for shooter velocity control. */
        public static final double kD = 0;
        /** Feedforward static friction constant */
        public static final double kShooterS = .2375;
        /** Feedforward velocity factor */
        public static final double kShooterV = .0172;
    }

    /**
     * Climber mechanism PID constants.
     */
    public static final class Climber {

        private Climber() {}

        /** Proportional gain for axle position control. */
        public static final double kP = 0;
        /** Integral gain for axle position control. */
        public static final double kI = 0;
        /** Derivative gain for axle position control. */
        public static final double kD = 0;
    }

    /**
     * PIDLerp's PID constants. Shared by {@link frc.robot.commands.drive.pathfind.PIDLerp},
     * {@link frc.robot.commands.drive.pathfind.Point}, and {@link frc.robot.commands.drive.PointAtHub}.
     */
    public static final class PIDLerp {

        private PIDLerp() {}

        /** Proportional gain for X and Y alignment errors. */
        public static final double kOrthoP = 4.75;
        /** Integral gain for alignment position errors. */
        public static final double kOrthoI = 0.00;
        /** Derivative gain for alignment position errors. */
        public static final double kOrthoD = 0.00;

        /** Proportional gain for alignment rotation error. */
        public static final double kTurnP = 7.00;
        /** Integral gain for alignment rotation error. */
        public static final double kTurnI = 0.00;
        /** Derivative gain for alignment rotation error. */
        public static final double kTurnD = 0.00;

        /** Position tolerance, meters (euclidian) */
        public static final double kPositionTolerance = 0.03;
        /** Rotational tolerance, radians */
        public static final double kRotationTolerance = 0.03;
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
        public static final double kOrthogonal = 1.8;
        /** Rotational rate limiting in percent/second (1.0=100%) */
        public static final double kRotational = 3.1;
    }
}
