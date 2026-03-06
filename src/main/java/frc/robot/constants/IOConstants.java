package frc.robot.constants;

import com.studica.frc.AHRS.NavXComType;

/**
 * Input/Output port assignments and hardware interface constants.
 *
 * <p>Contains CAN IDs, USB port assignments, and controller port mappings
 * for all robot hardware peripherals. Organized by subsystem for clarity.
 */
public final class IOConstants {

    private IOConstants() {}

    /**
     * Drivetrain hardware port assignments.
     *
     * <p>Currently contains only gyroscope configuration (commented out).
     * TODO: May need USB connection instead of MXP_SPI depending on NavX mounting.
     */
    public static final class Drivetrain {

        private Drivetrain() {}

        /** NavX gyro communication port type. */
        public static final NavXComType kGyroPort = NavXComType.kMXP_SPI;
    }

    /**
     * Shooter hardware port assignments.
     */
    public static final class Shooter {

        private Shooter() {}

        /** Leader shooter motor CAN ID. */
        public static final int kLeader = 50;

        /** Follower shooter motor CAN ID. */
        public static final int kFollower = 51;
    }

    /**
     * Spindexer hardware port assignments.
     */
    public static final class Spindexer {

        private Spindexer() {}

        /** SparkFlex (Neo Vortex) motor CAN ID. */
        public static final int kSpindexer = 52;

        /** SparkMax (Neo) motor CAN ID. */
        public static final int kKicker = 53;
    }

    /**
     * Intake hardware port assignments.
     */
    public static final class Intake {

        private Intake() {}

        /** Roller leader SparkFlex (Neo Vortex) CAN ID. */
        public static final int kRollerLeader = 54;

        /** Roller follower SparkFlex (Neo Vortex) CAN ID. */
        public static final int kRollerFollower = 55;

        /** Pivot leader SparkFlex (Neo Vortex) CAN ID. */
        public static final int kPivotLeader = 56;

        /** Pivot follower SparkFlex (Neo Vortex) CAN ID. */
        public static final int kPivotFollower = 57;
    }

    /**
     * Driver station controller port mappings and input filtering.
     *
     * <p>Defines USB ports for driver and operator controllers, plus
     * joystick deadband for eliminating stick drift.
     */
    public static final class Controller {

        private Controller() {}

        /** Driver controller USB port (primary pilot). */
        public static final int kDriver = 0;

        /** Operator controller USB port (secondary controls). */
        public static final int kOperator = 1;

        /** Keys for the driver paddles in order [BL, TL, TR, BR]. */
        public static final String kDriverPaddles = "team";

        /** Keys for the operator paddles in order [BL, TL, TR, BR]. */
        public static final String kOperatorPaddles = "2791";

        /**
         * Joystick deadband threshold.
         *
         * <p>Inputs within ±{@code 0.05} of center are treated as zero
         * to prevent unintended movement from controller drift.
         */
        public static final double kDeadband = 0.05;
    }
}
