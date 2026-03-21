package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.GameConstants;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import java.util.Arrays;
import java.util.Optional;

/**
 * Utility for handling alliance-based coordinate system transformations.
 *
 * <p>
 * In FRC, the field has a symmetric layout where:
 * <ul>
 *   <li>Blue alliance origin is at the field origin (0, 0)
 *   <li>Red alliance origin is at the opposite corner of the field
 * </ul>
 *
 * <p>
 * This class provides utilities to automatically flip poses and rotations
 * between alliance coordinate systems, allowing autonomous routines to
 * work symmetrically for both alliances.
 */
public class AllianceUtil {

    private AllianceUtil() {}

    /**
     * A container for the current alliance state with caching.
     *
     * <p>
     * This inner class caches the alliance once it's reported by FMS to avoid
     * repeated queries and handles the case where alliance information might not
     * be immediately available.
     */
    public static class AllianceCell {

        /** The cached alliance value. */
        private Alliance last = Alliance.Blue;

        /** Whether the alliance has been initialized from FMS. */
        private boolean wasInit = false;

        /** Private constructor for internal use only. */
        AllianceCell() {}

        /**
         * Gets the current alliance, with fallback to cached value.
         *
         * <p>
         * Queries FMS for the current alliance and caches the result. If FMS hasn't
         * reported yet and no prior value is cached, defaults to Blue and logs a warning.
         *
         * @return The current alliance (Blue or Red)
         */
        public DriverStation.Alliance get() {
            Optional<Alliance> current = DriverStation.getAlliance();

            if (current.isPresent() || wasInit) {
                last = current.orElse(last);
                wasInit = true;
                return last;
            }

            System.err.println(
                "WARNING: Tried alliance before FMS report. Defaulting to Blue."
            );
            Thread.dumpStack();
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "Alliance",
                    "Alliance not yet reported by FMS/DS; defaulting to Blue."
                )
            );

            return Alliance.Blue;
        }

        /**
         * Checks if the current alliance is Red.
         *
         * @return true if Red alliance, false if Blue alliance
         */
        public boolean invert() {
            return get() == DriverStation.Alliance.Red;
        }

        /**
         * Zero heading
         * @return the zero heading for the current alliance (facing opposing alliance)
         */
        public Rotation2d zero() {
            return autoflip(new Rotation2d());
        }

        /**
         * Automatically flips a pose if on Red alliance.
         *
         * @param pose The pose to potentially flip
         * @return The pose, flipped if Red alliance, unchanged if Blue alliance
         */
        public Pose2d autoflip(Pose2d pose) {
            return invert() ? AllianceUtil.flip(pose) : pose;
        }

        /**
         * Automatically flips an array of poses if on Red alliance.
         *
         * @param poses Array of poses to potentially flip
         * @return Array of poses, all flipped if Red alliance, unchanged if Blue alliance
         */
        public Pose2d[] autoflip(Pose2d[] poses) {
            return invert() ? AllianceUtil.flip(poses) : poses;
        }

        /**
         * Automatically flips a rotation if on Red alliance.
         *
         * @param rotation The rotation to potentially flip
         * @return The rotation, flipped if Red alliance, unchanged if Blue alliance
         */
        public Rotation2d autoflip(Rotation2d rotation) {
            return invert() ? AllianceUtil.flip(rotation) : rotation;
        }

        public Translation2d autoflip(Translation2d translation) {
            return invert() ? AllianceUtil.flip(translation) : translation;
        }

        /**
         * Automatically flips an array of rotations if on Red alliance.
         *
         * @param rotations Array of rotations to potentially flip
         * @return Array of rotations, all flipped if Red alliance, unchanged if Blue alliance
         */
        public Rotation2d[] autoflip(Rotation2d[] rotations) {
            return invert() ? AllianceUtil.flip(rotations) : rotations;
        }

        /**
         * Gets the sign multiplier for the current alliance.
         *
         * @return 1 for Blue alliance, -1 for Red alliance
         */
        public int sign() {
            return invert() ? -1 : 1;
        }
    }

    /**
     * Global alliance cell instance for convenient access to alliance information.
     *
     * SAFETY: This assumes FMS/DS has been connected. If it hasn't been connected yet,
     * this defaults to Blue alliance and logs a warning.
     */
    public static final AllianceCell unsafe = new AllianceCell();

    /**
     * Checks if the current alliance is Red.
     *
     * @return Optional containing true if Red, false if Blue, empty if FMS/DS hasn't reported
     */
    public static Optional<Boolean> invert() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.<Boolean>map(x -> x == DriverStation.Alliance.Red);
    }

    /**
     * Flips a pose from one alliance's coordinate system to the other.
     *
     * This mirrors the pose across the field's centerline to convert between
     * Blue and Red coordinate frames.
     *
     * @param pose The pose to flip
     * @return The flipped pose in the other alliance's coordinate system
     */
    public static Pose2d flip(Pose2d pose) {
        return pose.relativeTo(GameConstants.kRedOrigin);
    }

    /**
     * Flips an array of poses from one alliance's coordinate system to the other.
     *
     * @param poses The array of poses to flip
     * @return The array of flipped poses in the other alliance's coordinate system
     */
    public static Pose2d[] flip(Pose2d[] poses) {
        return Arrays.stream(poses)
            .map(AllianceUtil::flip)
            .toArray(Pose2d[]::new);
    }

    /**
     * Flips a rotation from one alliance's coordinate system to the other.
     *
     * Flipped rotation is the original rotation plus 180 degrees (π radians).
     *
     * @param rotation The rotation to flip
     * @return The flipped rotation (rotated by 180 degrees)
     */
    public static Rotation2d flip(Rotation2d rotation) {
        return MathPlus.normalizeAngle(rotation.plus(Rotation2d.kPi));
    }

    /**
     * Flips an array of rotations from one alliance's coordinate system to the other.
     *
     * @param rotations The array of rotations to flip
     * @return The array of flipped rotations (each rotated by 180 degrees)
     */
    public static Rotation2d[] flip(Rotation2d[] rotations) {
        return Arrays.stream(rotations)
            .map(AllianceUtil::flip)
            .toArray(Rotation2d[]::new);
    }

    /**
     * Automatically flips a pose if the current alliance is Red.
     *
     * @param pose The pose to conditionally flip
     * @return Optional containing the pose (flipped if Red, unchanged if Blue)
     */
    public static Optional<Pose2d> autoflip(Pose2d pose) {
        return invert().map(x -> x ? flip(pose) : pose);
    }

    /**
     * Automatically flips a rotation if the current alliance is Red.
     *
     * @param rotation The rotation to conditionally flip
     * @return Optional containing the rotation (flipped if Red, unchanged if Blue)
     */
    public static Optional<Rotation2d> autoflip(Rotation2d rotation) {
        return invert().map(x -> x ? flip(rotation) : rotation);
    }

    /**
     * Automatically flips an array of poses if the current alliance is Red.
     *
     * @param poses The array of poses to conditionally flip
     * @return Optional containing the array of poses (flipped if Red, unchanged if Blue
     */
    public static Optional<Pose2d[]> autoflip(Pose2d[] poses) {
        return invert().map(x -> x ? flip(poses) : poses);
    }

    /**
     * Gets the sign multiplier for the current alliance.
     *
     * Useful for equations that need to scale values by alliance (-1 for Red, 1 for Blue).
     *
     * @return Optional containing 1 for Blue alliance, -1 for Red alliance
     */
    public static Optional<Integer> sign() {
        return invert().map(x -> x ? -1 : 1);
    }

    public static Translation2d flip(Translation2d t2d) {
        return new Translation2d(
            GameConstants.kFieldLength - t2d.getX(),
            GameConstants.kFieldWidth - t2d.getY()
        );
    }

    public static Optional<Translation2d> autoflip(Translation2d t2d) {
        return invert().map(x -> x ? flip(t2d) : t2d);
    }
}
