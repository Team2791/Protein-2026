package frc.robot.commands.drive.pathfind;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract command for pose-based robot navigation using holonomic drive control.
 *
 * <p>
 * This is an abstract command that subclasses override via {@link Supplied} or
 * by implementing {@link #target()}.
 *
 * <p>
 * Default concrete implementation: {@link Supplied}
 */
public abstract class Pathfind extends WrapperCommand {

    static final String KEY_TARGET = "Pathfind/Target";

    /**
     * Concrete {@link Pathfind} implementation that accepts a target pose via Supplier.
     *
     * <p>
     * Useful for dynamic targets that may change during the command, or for
     * creating {@link Pathfind} instances with a fixed target pose.
     */
    public static class Supplied extends Pathfind {

        /** Supplier for the target pose (can be fixed or dynamic). */
        final Supplier<Pose2d> target;

        /**
         * Constructs {@link Pathfind} with a dynamic target supplier.
         *
         * @param drive Drive subsystem for movement control
         * @param target Supplier providing target pose each execute cycle
         */
        public Supplied(Drive drive, Supplier<Pose2d> target) {
            super(drive);
            this.target = target;
        }

        /**
         * Constructs {@link Pathfind} with a fixed target pose.
         *
         * @param drive Drive subsystem for movement control
         * @param target Fixed target pose for this navigation
         */
        public Supplied(Drive drive, Pose2d target) {
            this(drive, () -> target);
        }

        @Override
        protected Pose2d target() {
            return target.get();
        }
    }

    /** Cached target pose for this navigation. */
    Pose2d cache;

    final Drive drive;

    /**
     * Constructs a Navigate command.
     *
     * <ol>
     *  <li>Calls {@link #target()} to get the target pose and caches it.
     *  <li>Logs the target pose to the field for visualization.
     *  <li>Runs {@link Repulse} for faraway pathfinding.
     *  <li>Runs {@link Nearby} for close-range precision movement.
     *  <li>Clears the target pose from the field logs.
     * </ol>
     *
     * @param drive Drive subsystem for movement control
     */
    public Pathfind(Drive drive) {
        super(new SequentialCommandGroup());
        this.drive = drive;

        // Due to how Java works, we can't call add these commands in the `super()`
        // call since we cannot reference `this` before the superclass constructor.
        //
        // Instead, we wrap an empty SequentialCommandGroup and add the commands afterward.
        // Since the wrapped `m_command` is stored as a plain Command, we need to cast back
        // to SequentialCommandGroup to call addCommands.
        ((SequentialCommandGroup) super.m_command).addCommands(
            new Repulse(drive, this::currentTarget),
            new Nearby(drive, this::currentTarget)
        );
    }

    @Override
    public void initialize() {
        cache = target();
        drive.field.getObject(KEY_TARGET).setPose(cache);
        Logger.recordOutput(KEY_TARGET, cache);

        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        Pose2d invalid = new Pose2d(-1, -1, new Rotation2d());
        drive.field.getObject(KEY_TARGET).setPose(invalid);
        Logger.recordOutput(KEY_TARGET, invalid);
    }

    /**
     * Gets the target pose for this navigation.
     *
     * <p>
     * Subclasses must implement this to provide the target, which may be
     * constant or dynamic (e.g., based on vision measurements).
     *
     * @return Target pose, or null if no valid target
     */
    protected abstract Pose2d target();

    /**
     * Gets the currently cached target pose.
     * @return The cached target pose
     */
    protected final Pose2d currentTarget() {
        return cache;
    }
}
