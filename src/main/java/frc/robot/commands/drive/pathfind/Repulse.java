package frc.robot.commands.drive.pathfind;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Repulsor;
import frc.robot.util.Vec2;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Faraway pathfinding command.
 *
 * <p>
 * Uses {@link Repulsor} for faraway pathfinding to a target pose. Repulsor works
 * by generating a potential field around obstacles and the goal, then using
 * the laws of physics to "push" the robot away from obstacles and "pull" it
 * toward the goal.
 *
 * <p>
 * This command is used as the first step in {@link Pathfind}: far from the target,
 * use Repulsor to pathfind around obstacles. When close to the target, switch to
 * {@link Nearby} for precision movement.
 */
class Repulse extends Command {

    static final String KEY_TRAJ = "Pathfind/Repulse/Trajectory";
    static final String KEY_TARGET = "Pathfind/Repulse/Target";

    /** Supplier for the target pose. */
    final Supplier<Pose2d> target;

    /** Repulsor instance for pathfinding calculations. */
    final Repulsor repulsor = new Repulsor();

    final Drive drive;

    /** PID controller for orthogonal movement correction. */
    final PIDController orthoController = new PIDController(
        ControlConstants.Repulsor.kOrthoP,
        ControlConstants.Repulsor.kOrthoI,
        ControlConstants.Repulsor.kOrthoD
    );

    /** PID controller for rotational movement correction. */
    final PIDController rotController = new PIDController(
        ControlConstants.Repulsor.kTurnP,
        ControlConstants.Repulsor.kTurnI,
        ControlConstants.Repulsor.kTurnD
    );

    /** Currently targeted pose, reset every {@link #initialize()} */
    Pose2d currentTarget;

    /**
     * Constructor for Repulse command.
     *
     * @param drive Drive subsystem for motion control
     * @param target Supplier providing the target pose
     */
    Repulse(Drive drive, Supplier<Pose2d> target) {
        this.target = target;
        this.drive = drive;

        addRequirements(drive);
    }

    /**
     * Initializes the Repulse command.
     *
     * <ol>
     *  <li>Gets the current target pose from the supplier.
     *  <li>Sets the Repulsor goal to the target translation.
     *  <li>Logs the planned trajectory for visualization.
     * </ol>
     */
    @Override
    public void initialize() {
        currentTarget = target.get();
        repulsor.setGoal(currentTarget.getTranslation());

        List<Translation2d> trj = repulsor.getTrajectory(
            drive.getPose().getTranslation(),
            currentTarget.getTranslation(),
            0.1 // 10cm steps
        );

        Pose2d[] poses = trj
            .stream()
            .map(t -> new Pose2d(t, new Rotation2d()))
            .toArray(Pose2d[]::new);

        Logger.recordOutput(KEY_TRAJ, poses);
        drive.field.getObject(KEY_TRAJ).setPoses(poses);
    }

    /**
     * Runs the Repulse pathfinding logic each cycle.
     *
     * <ol>
     *  <li>Gets the next SwerveSample from Repulsor based on current pose and velocity.
     *  <li>Calculates the distance vector to the target.
     *  <li>Computes PID correction power based on distance.
     *  <li>Adds PID correction to Repulsor's suggested velocities.
     *  <li>Drives the drive with the combined velocities and rotational PID output.
     * </ol>
     */
    @Override
    public void execute() {
        SwerveSample sample = repulsor.getCmd(
            drive.getPose(),
            drive.getChassisSpeeds(),
            ControlConstants.Drivetrain.MaxSpeed.kLinear,
            true
        );

        Logger.recordOutput(KEY_TARGET, sample.getPose());
        drive.field.getObject(KEY_TARGET).setPose(currentTarget);

        Vec2 pose = new Vec2(drive.getPose());
        Vec2 target = new Vec2(sample.getPose());
        Vec2 dist = target.sub(pose);
        Vec2 pidpower = dist.norm().mul(orthoController.calculate(dist.mag()));
        Vec2 result = new Vec2(sample.vx, sample.vy).add(pidpower);

        drive.drive(
            new ChassisSpeeds(
                result.x,
                result.y,
                rotController.calculate(
                    drive.getPose().getRotation().getRadians(),
                    currentTarget.getRotation().getRadians()
                )
            )
        );
    }

    /**
     * Cleans up after the Repulse command ends.
     *
     * <p>
     * Clears the target and trajectory logs from the field visualization.
     */
    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput(KEY_TARGET, new Pose2d(-1, -1, new Rotation2d()));
        Logger.recordOutput(KEY_TRAJ, new Pose2d[0]);
        drive.field.getObject(KEY_TRAJ).setPoses(new Pose2d[0]);
    }

    /**
     * Determines if the Repulse command is finished.
     *
     * <p>
     * Finishes when the robot is within the nearby threshold distance of the target.
     * See {@link ControlConstants.PIDLerp#kNearbyThreshold}.
     */
    @Override
    public boolean isFinished() {
        Vec2 pose = new Vec2(drive.getPose());
        Vec2 target = new Vec2(currentTarget);
        Vec2 dist = target.sub(pose);

        return dist.mag() < ControlConstants.Repulsor.kNearbyThreshold;
    }
}
