package frc.robot.constants;

import static frc.robot.util.MathPlus.kTau;

import edu.wpi.first.math.util.Units;
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
     * Intake mechanism PID constants.
     */
    public static final class Intake {

        private Intake() {}

        /** Proportional gain for roller velocity control. */
        public static final double kRollerP = 0;
        /** Integral gain for roller velocity control. */
        public static final double kRollerI = 0;
        /** Derivative gain for roller velocity control. */
        public static final double kRollerD = 0;

        /** Proportional gain for pivot position control. */
        public static final double kPivotP = 0.5;
        /** Integral gain for pivot position control. */
        public static final double kPivotI = 0;
        /** Derivative gain for pivot position control. */
        public static final double kPivotD = 0;

        public static final double kPivotS = 0.21;
        public static final double kPivotG = 1.00;
        public static final double kPivotZero = Units.degreesToRadians(78.8);
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
     * PIDLerp's PID constants.
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
        public static final double kTurnP = 2.75;
        /** Integral gain for alignment rotation error. */
        public static final double kTurnI = 0.00;
        /** Derivative gain for alignment rotation error. */
        public static final double kTurnD = 0.04;

        /** Position tolerance, meters (euclidian) */
        public static final double kPositionTolerance = 0.005;
        /** Rotational tolerance, radians */
        public static final double kRotationTolerance = 0.03;
    }

    /**
     * Repulsor's PID constants
     */
    public static final class Repulsor {

        /** Proportional gain for X and Y alignment errors. */
        public static final double kOrthoP = 4.75;
        /** Integral gain for alignment position errors. */
        public static final double kOrthoI = 0.00;
        /** Derivative gain for alignment position errors. */
        public static final double kOrthoD = 0.00;

        /** Proportional gain for alignment rotation error. */
        public static final double kTurnP = 2.75;
        /** Integral gain for alignment rotation error. */
        public static final double kTurnI = 0.00;
        /** Derivative gain for alignment rotation error. */
        public static final double kTurnD = 0.04;

        /** Threshold to switch to nearby control, meters (euclidian) */
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
        public static final double kTurnP = 2.75;
        /** Integral gain for alignment rotation error. */
        public static final double kTurnI = 0.00;
        /** Derivative gain for alignment rotation error. */
        public static final double kTurnD = 0.04;

        /** Maximum angular velocity during alignment (radians/second). TODO: May be increased. */
        public static final double kMaxTurnVelocity = kTau;
        /** Maximum angular acceleration during alignment (radians/second²). */
        public static final double kMaxTurnAcceleration = kTau;

        /** Position tolerance, meters (euclidian) */
        public static final double kPositionTolerance = 0.005;
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
