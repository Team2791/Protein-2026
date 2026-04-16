package frc.robot.commands.drive.pathfind;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil;

/**
 * Instantly resets the robot's odometry to a known pose.
 *
 * <p>This is an instant command (finishes on the same cycle it starts) used at the
 * beginning of an autonomous routine to align the robot's internal pose estimate with
 * where it was physically placed on the field.
 *
 * <p>The pose is supplied in blue-alliance coordinates. {@link AllianceUtil#autoflip}
 * mirrors it to the red side when needed, so the same {@link AutoTask} constant works
 * on both alliances without any caller-side flipping.
 *
 * <p>Requires {@link Drive} to ensure no other command is driving while the pose is
 * being overwritten.
 *
 * @see frc.robot.auto.AutoTask
 */
public class ResetPose extends Command {

    final Drive drive;

    /** Blue-alliance reference pose. Flipped to field coordinates in {@link #initialize()}. */
    final Pose2d blue;

    /**
     * @param drive     drive subsystem
     * @param bluePose  target pose in blue-alliance coordinates
     */
    public ResetPose(Drive drive, Pose2d bluePose) {
        this.drive = drive;
        this.blue = bluePose;

        addRequirements(drive);
    }

    /**
     * Flips {@link #blue} to the correct alliance side and writes it to odometry.
     * Falls back to the original blue pose if no alliance is set.
     */
    @Override
    public void initialize() {
        drive.setPose(AllianceUtil.autoflip(blue).orElse(blue));
    }

    /** Always returns {@code true} — this command completes in a single cycle. */
    @Override
    public boolean isFinished() {
        return true;
    }
}
