package frc.robot.commands.drive.pathfind;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;

/**
 * Drives to a target translation while holding a fixed heading throughout transit.
 *
 * <p>Extends {@link PIDLerp} and uses a {@link Point} instance as a heading controller
 * that runs <em>inline</em> — it is never scheduled as a command and never registers
 * a requirement on {@link frc.robot.subsystems.drive.Drive}. Each cycle,
 * {@link Point#calculate()} produces a rotation output that is injected directly into
 * {@link PIDLerp#execute(double)}, so translation and rotation are sent to the drive
 * in a single {@code ChassisSpeeds} packet.
 *
 * <p>This is the correct way to combine translation and rotation without a WPILib
 * subsystem conflict. If two independent commands each required {@code Drive}, the
 * scheduler would cancel one of them.
 *
 * <p>Finishes when <em>both</em> the translation ({@link PIDLerp#isFinished()}) and
 * rotation ({@link Point#isFinished()}) are settled. This prevents the robot from
 * arriving at the target position while still spinning.
 *
 * <p>The {@code locked} rotation must be expressed in blue-alliance coordinates.
 * Use {@link frc.robot.util.AllianceUtil.unsafe#autoflip} to convert if needed.
 *
 * <p>Requires {@link Drive} (inherited from {@link PIDLerp}).
 *
 * @see Pathfind
 * @see PIDLerp
 * @see Point
 */
public class LockedPIDLerp extends PIDLerp {

    /**
     * The rotation controller used to hold {@link #lock} during transit.
     *
     * <p>Not scheduled as a WPILib command — driven inline via {@link Point#calculate()}
     * to avoid a subsystem requirement conflict with {@link PIDLerp}.
     */
    final Point headless;

    /** The target heading to hold throughout transit, in blue-alliance coordinates. */
    final Rotation2d lock;

    /**
     * Creates a LockedPIDLerp at full drive speed.
     *
     * @param drive  drive subsystem
     * @param target blue-alliance target pose (rotation field is ignored; use {@code locked})
     * @param locked heading to hold during transit, in blue-alliance coordinates
     */
    public LockedPIDLerp(Drive drive, Pose2d target, Rotation2d locked) {
        super(drive, target);
        this.headless = new Point(drive, locked);
        this.lock = locked;
    }

    /**
     * Creates a LockedPIDLerp with a capped translation speed.
     *
     * @param drive  drive subsystem
     * @param target blue-alliance target pose (rotation field is ignored; use {@code locked})
     * @param locked heading to hold during transit, in blue-alliance coordinates
     * @param maxVel maximum translation speed in m/s
     */
    public LockedPIDLerp(
        Drive drive,
        Pose2d target,
        Rotation2d locked,
        double maxVel
    ) {
        super(drive, target, maxVel);
        this.headless = new Point(drive, locked);
        this.lock = locked;
    }

    /**
     * Resets both the translation PID controllers (via {@link PIDLerp#initialize()})
     * and the rotation PID controller (via {@link Point#initialize()}).
     */
    @Override
    public void initialize() {
        super.initialize();
        headless.initialize();
    }

    /**
     * Runs one cycle of translation control, injecting the current rotation output
     * from {@link Point#calculate()} so both axes are commanded atomically.
     */
    @Override
    public void execute() {
        super.execute(headless.calculate());
    }

    /**
     * Stops the drive (via {@link PIDLerp#end(boolean)}) and sends a final stop
     * to clear any residual rotation output from the heading controller.
     */
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        headless.end(interrupted);
    }

    /**
     * Finishes when translation has reached the target ({@link PIDLerp#isFinished()})
     * <em>and</em> the heading is settled ({@link Point#isFinished()}).
     *
     * <p>Both conditions are required so the robot does not advance to the next step
     * while still spinning toward the locked heading.
     */
    @Override
    public boolean isFinished() {
        return super.isFinished() && headless.isFinished();
    }
}
