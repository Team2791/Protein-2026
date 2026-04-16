package frc.robot.commands.drive.pathfind;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

/**
 * Convenience facade that drives to a target pose: first translating (with optional heading
 * lock), then snapping to the target's final rotation.
 *
 * <p>Always a two-step sequence:
 * <ol>
 *   <li><b>Translation phase</b> — {@link PIDLerp} or {@link LockedPIDLerp}, depending on
 *       whether a {@code lock} rotation is provided. During this phase the robot moves
 *       toward the target position; if a lock is given it also holds that heading
 *       throughout.
 *   <li><b>Rotation snap</b> — {@link Point} rotates to {@code target.getRotation()}.
 *       This ensures the robot arrives at the correct final heading regardless of what
 *       the lock was during transit.
 * </ol>
 *
 * <p>All poses must be in blue-alliance coordinates. Use
 * {@link frc.robot.util.AllianceUtil.unsafe#autoflip} on the caller side if needed.
 *
 * <p>Requires {@link Drive} throughout (both sub-commands require it sequentially,
 * so there is no conflict).
 *
 * <h3>Static factories</h3>
 * {@link #targetLock(Drive, Pose2d)} and {@link #targetLock(Drive, Pose2d, double)} are
 * the most common entry points from {@link frc.robot.auto.AutoTask}: they lock the heading
 * to {@code target.getRotation()}, so the robot faces the final direction for the entire
 * approach and simply confirms it at the end.
 *
 * @see PIDLerp
 * @see LockedPIDLerp
 * @see Point
 */
public class Pathfind extends SequentialCommandGroup {

    /**
     * Drives to {@code target} at full speed, then snaps to {@code target}'s rotation.
     * No heading lock during transit — the robot may spin freely.
     *
     * @param drive  drive subsystem
     * @param target blue-alliance target pose
     */
    public Pathfind(Drive drive, Pose2d target) {
        addCommands(
            new PIDLerp(drive, target),
            new Point(drive, target.getRotation())
        );
    }

    /**
     * Drives to {@code target} at a capped speed, then snaps to {@code target}'s rotation.
     * No heading lock during transit.
     *
     * @param drive  drive subsystem
     * @param target blue-alliance target pose
     * @param maxvel maximum translation speed in m/s
     */
    public Pathfind(Drive drive, Pose2d target, double maxvel) {
        addCommands(
            new PIDLerp(drive, target, maxvel),
            new Point(drive, target.getRotation())
        );
    }

    /**
     * Drives to {@code target} at full speed while holding {@code lock} throughout transit,
     * then snaps to {@code target}'s final rotation.
     *
     * @param drive  drive subsystem
     * @param target blue-alliance target pose
     * @param lock   heading to hold during the translation phase, in blue-alliance coordinates
     */
    public Pathfind(Drive drive, Pose2d target, Rotation2d lock) {
        addCommands(
            new LockedPIDLerp(drive, target, lock),
            new Point(drive, target.getRotation())
        );
    }

    /**
     * Drives to {@code target} at a capped speed while holding {@code lock} throughout transit,
     * then snaps to {@code target}'s final rotation.
     *
     * @param drive  drive subsystem
     * @param target blue-alliance target pose
     * @param lock   heading to hold during the translation phase, in blue-alliance coordinates
     * @param maxvel maximum translation speed in m/s
     */
    public Pathfind(
        Drive drive,
        Pose2d target,
        Rotation2d lock,
        double maxvel
    ) {
        addCommands(
            new LockedPIDLerp(drive, target, lock, maxvel),
            new Point(drive, target.getRotation())
        );
    }

    /**
     * Creates a {@code Pathfind} that faces the target heading for the entire approach.
     *
     * <p>Equivalent to {@code new Pathfind(drive, target, target.getRotation())}: the robot
     * locks to {@code target.getRotation()} during transit and then confirms that heading
     * with a final {@link Point} snap. This is the standard autonomous navigation call.
     *
     * @param drive  drive subsystem
     * @param target blue-alliance target pose; its rotation is used as both the lock and the final heading
     * @return a new {@link Pathfind} with a heading lock equal to {@code target.getRotation()}
     */
    public static Pathfind targetLock(Drive drive, Pose2d target) {
        return new Pathfind(drive, target, target.getRotation());
    }

    /**
     * Creates a {@code Pathfind} that faces the target heading for the entire approach,
     * with a capped translation speed.
     *
     * @param drive  drive subsystem
     * @param target blue-alliance target pose; its rotation is used as both the lock and the final heading
     * @param maxvel maximum translation speed in m/s
     * @return a new {@link Pathfind} with a heading lock and capped speed
     */
    public static Pathfind targetLock(
        Drive drive,
        Pose2d target,
        double maxvel
    ) {
        return new Pathfind(drive, target, target.getRotation(), maxvel);
    }
}
