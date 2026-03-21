package frc.robot.auto.generated;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final Distance start_line = Units.Meters.of(4.003);
    public static final Distance start_x = Units.Meters.of(3.556);
    public static final Distance bot_rad = Units.Meters.of(0.448);
    public static final Distance trench_rhs_center = Units.Meters.of(0.634);
    public static final Distance trench_lhs_center = Units.Meters.of(7.435);
    public static final Distance hub_y = Units.Meters.of(4.035);
    public static final Distance pad = Units.Meters.of(0.051);
    public static final Distance center_x = Units.Meters.of(8.27);
    public static final Distance balls_rad = Units.Meters.of(2.31);
    public static final Distance outpost_y = Units.Meters.of(0.666);

    public static final class Poses {
        public static final Pose2d pos_3 = new Pose2d(3.556, 0.634, Rotation2d.fromRadians(1.571));
        public static final Pose2d pos_1 = new Pose2d(3.556, 7.435, Rotation2d.fromRadians(-1.571));
        public static final Pose2d pos_2 = new Pose2d(3.556, 4.035, Rotation2d.kZero);
        public static final Pose2d balls_rhs = new Pose2d(7.772, 1.226, Rotation2d.fromRadians(3.142));
        public static final Pose2d balls_lhs = new Pose2d(7.772, 6.843, Rotation2d.kZero);
        public static final Pose2d center_rhs = new Pose2d(7.772, 3.536, Rotation2d.fromRadians(3.142));
        public static final Pose2d center_lhs = new Pose2d(7.772, 4.533, Rotation2d.kZero);
        public static final Pose2d trench_score = new Pose2d(4.451, 0.634, Rotation2d.fromRadians(1.571));
        public static final Pose2d hub_score = new Pose2d(2.213, 4.035, Rotation2d.kZero);
        public static final Pose2d outpost = new Pose2d(0.498, 0.666, Rotation2d.fromRadians(1.571));
        public static final Pose2d outpost_score = new Pose2d(0.946, 0.869, Rotation2d.fromRadians(1.571));

        private Poses() {}
    }

    private ChoreoVars() {}
}