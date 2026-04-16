package frc.robot.commands.drive.pathfind;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AStar;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Vec2;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

/**
 * Drives the robot along an obstacle-avoiding path to a target translation.
 *
 * <p>Uses two independent {@link PIDController}s (X and Y) to command field-relative
 * chassis speeds. The path is computed once in {@link #initialize()} via {@link AStar},
 * then tracked incrementally: each cycle, {@link #search()} advances a "lookahead" index
 * as far along the path as a clear line-of-sight raycast allows, so the robot always
 * aims at the farthest visible waypoint rather than following the path blindly.
 *
 * <p>This command handles <em>translation only</em>. Rotation must be handled separately:
 * <ul>
 *   <li>Use {@link LockedPIDLerp} to hold a fixed heading <em>during</em> transit.
 *   <li>Use {@link Pathfind} to snap to the target's rotation <em>after</em> transit.
 * </ul>
 *
 * <p>All poses are computed in blue-alliance coordinates via
 * {@link AllianceUtil.unsafe#autoflip}; the result is automatically mirrored for red.
 *
 * <p>Requires {@link Drive}.
 *
 * @see LockedPIDLerp
 * @see Pathfind
 */
public class PIDLerp extends Command {

    /**
     * Pair of SmartDashboard-tunable PID controllers for field-relative X and Y translation.
     *
     * <p>Both axes use the same gains from {@link ControlConstants.PIDLerp} but are separate
     * objects so that WPILib's SmartDashboard tuning panel can display their individual errors.
     * Gains published to SmartDashboard as {@code PID/LerpX} and {@code PID/LerpY}.
     */
    static class TunablePID {

        final PIDController xctl = new PIDController(
            ControlConstants.PIDLerp.kOrthoP,
            ControlConstants.PIDLerp.kOrthoI,
            ControlConstants.PIDLerp.kOrthoD
        );

        final PIDController yctl = new PIDController(
            ControlConstants.PIDLerp.kOrthoP,
            ControlConstants.PIDLerp.kOrthoI,
            ControlConstants.PIDLerp.kOrthoD
        );

        public TunablePID() {
            SmartDashboard.putData("PID/LerpX", xctl);
            SmartDashboard.putData("PID/LerpY", yctl);
        }

        /**
         * Calculates the commanded velocity vector from {@code current} to {@code target}.
         *
         * <p>The result is negated on red alliance so that the field-relative direction is
         * preserved after the coordinate flip applied by {@link AllianceUtil.unsafe#autoflip}.
         *
         * @param current robot's current blue-frame position
         * @param target  desired blue-frame position
         * @return field-relative velocity command as a {@link Vec2}
         */
        public Vec2 commanded(Vec2 current, Vec2 target) {
            double x = xctl.calculate(current.x, target.x);
            double y = yctl.calculate(current.y, target.y);
            Vec2 ret = new Vec2(x, y);

            if (AllianceUtil.unsafe.invert()) return ret.neg();
            else return ret;
        }

        /**
         * @return true if both X and Y controllers are within their configured tolerance
         */
        public boolean atTarget() {
            return xctl.atSetpoint() && yctl.atSetpoint();
        }
    }

    /** Blue-alliance target pose (translation used; rotation ignored by this command). */
    final Pose2d target;
    final Drive drive;

    /** Maximum output speed in m/s; clamped after PID calculation. */
    final double maxvel;

    final TunablePID ctl = new TunablePID();

    /**
     * Waypoints computed by A* from the robot's position to {@link #target}.
     * Recomputed each time the command initializes.
     */
    Vec2[] path = new Vec2[0];

    /**
     * Index of the farthest waypoint in {@link #path} that is currently reachable
     * by a clear line-of-sight raycast. Advanced forward each cycle by {@link #search()}.
     */
    int farthest = 0;

    /**
     * Creates a PIDLerp command at full drive speed.
     *
     * @param drive  drive subsystem
     * @param target blue-alliance target pose (rotation field is ignored)
     */
    public PIDLerp(Drive drive, Pose2d target) {
        this(drive, target, DriveConstants.maxSpeedMetersPerSec);
    }

    /**
     * Creates a PIDLerp command with a capped translation speed.
     *
     * @param drive  drive subsystem
     * @param target blue-alliance target pose (rotation field is ignored)
     * @param maxvel maximum translation speed in m/s
     */
    public PIDLerp(Drive drive, Pose2d target, double maxvel) {
        this.drive = drive;
        this.target = target;
        this.maxvel = maxvel;

        addRequirements(drive);
    }

    /** @return robot's current position in blue-alliance coordinates */
    Vec2 blue() {
        return new Vec2(AllianceUtil.unsafe.autoflip(drive.getPose()));
    }

    /**
     * Advances {@link #farthest} forward along {@link #path} as far as a clear
     * line-of-sight raycast from the robot's current position allows.
     *
     * <p>On each call, the index increments until a waypoint fails the raycast
     * (i.e. an obstacle lies between the robot and that waypoint), then clamps
     * to the last valid index. This ensures the robot always aims at the most
     * "advanced" reachable point rather than re-traversing already-cleared segments.
     */
    void search() {
        Vec2 us = blue();

        for (; farthest < path.length; farthest++) {
            Vec2 tt = path[farthest];
            boolean ok = AStar.raycast(us, tt);
            if (!ok) break;
        }

        farthest = Math.min(farthest, path.length - 1);
    }

    /** @return true if the lookahead index has reached the final waypoint */
    boolean atlast() {
        return farthest == path.length - 1;
    }

    /**
     * Computes the A* path from the robot's current position to {@link #target},
     * resets both PID controllers, and logs the planned path for visualization.
     */
    @Override
    public void initialize() {
        ctl.xctl.reset();
        ctl.yctl.reset();

        Vec2 bot = blue();
        this.path = AStar.path(bot, new Vec2(target));

        Logger.recordOutput(
            "PIDLerp/Direct",
            new Translation2d[] { bot.wpi(), target.getTranslation() }
        );

        drive.field
            .getObject("PIDLerp/Direct")
            .setPoses(new Pose2d[] { bot.wpi(new Rotation2d()), target });

        Logger.recordOutput(
            "PIDLerp/Path",
            Arrays.stream(this.path)
                .map(v -> v.wpi())
                .toArray(Translation2d[]::new)
        );

        drive.field
            .getObject("PIDLerp/Path")
            .setPoses(
                Arrays.stream(this.path)
                    .map(v -> v.wpi(new Rotation2d()))
                    .toList()
            );
    }

    /**
     * Drives translation-only (zero rotation). Delegates to {@link #execute(double)}.
     */
    @Override
    public void execute() {
        execute(0);
    }

    /**
     * Advances the lookahead, computes PID translation speeds, clamps to {@link #maxvel},
     * and sends {@link ChassisSpeeds} to the drive with the given rotation component.
     *
     * <p>Package-private so that {@link LockedPIDLerp} can inject a rotation
     * output from {@link Point#calculate()} into the same {@code drive.drive()} call,
     * ensuring translation and rotation are always commanded in a single packet.
     *
     * @param withRotation rotation speed in rad/s (0 for translation-only)
     */
    void execute(double withRotation) {
        search();

        Vec2 bot = blue();
        Vec2 target = atlast() ? new Vec2(this.target) : path[farthest];
        Vec2 speeds = ctl.commanded(bot, target);

        if (speeds.mag() > maxvel) {
            speeds = speeds.norm().mul(maxvel);
        }

        Logger.recordOutput(
            "PIDLerp/CurrentLerp",
            new Translation2d[] { bot.wpi(), target.wpi() }
        );

        drive.field
            .getObject("PIDLerp/CurrentLerp")
            .setPoses(
                new Pose2d[] {
                    bot.wpi(new Rotation2d()),
                    target.wpi(new Rotation2d()),
                }
            );

        drive.drive(new ChassisSpeeds(speeds.x, speeds.y, withRotation));
    }

    /** Stops the drive and clears all visualization objects. */
    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds());

        drive.field.getObject("PIDLerp/Direct").setPoses();
        drive.field.getObject("PIDLerp/CurrentLerp").setPoses();
        drive.field.getObject("PIDLerp/Path").setPoses();
    }

    /**
     * Finishes when the robot is within {@link ControlConstants.PIDLerp#kPositionTolerance}
     * of the target, the lookahead has reached the final waypoint, and chassis speed
     * has decayed below 0.1 m/s.
     *
     * <p>All three conditions are required to prevent premature termination: the position
     * check guards against early exit far from the goal, the {@link #atlast()} check
     * guards against exit while still routing around an obstacle, and the speed check
     * guards against exit while the robot is still coasting.
     */
    @Override
    public boolean isFinished() {
        double posErr = blue().sub(new Vec2(target)).mag();

        return (
            posErr < ControlConstants.PIDLerp.kPositionTolerance &&
            atlast() &&
            new Vec2(drive.getChassisSpeeds()).mag() < 0.1
        );
    }
}
