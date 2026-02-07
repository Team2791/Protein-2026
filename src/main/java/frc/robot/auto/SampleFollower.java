package frc.robot.auto;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/**
 * Sample follower for testing and tuning.
 *
 * <p>Follows a SwerveSample by using PID controllers to track the sample's pose. Used for
 * testing and tuning PID constants for autonomous path following.
 */
public class SampleFollower {

    /** Drive subsystem for motion control. */
    final Drive drive;

    /** PID controller for X-axis position tracking. */
    final PIDController xCtl = new PIDController(
        ControlConstants.Auto.kOrthoP,
        ControlConstants.Auto.kOrthoI,
        ControlConstants.Auto.kOrthoD
    );

    /** PID controller for Y-axis position tracking. */
    final PIDController yCtl = new PIDController(
        ControlConstants.Auto.kOrthoP,
        ControlConstants.Auto.kOrthoI,
        ControlConstants.Auto.kOrthoD
    );

    /** PID controller for rotational position tracking. */
    final PIDController rotCtl = new PIDController(
        ControlConstants.Auto.kTurnP,
        ControlConstants.Auto.kTurnI,
        ControlConstants.Auto.kTurnD
    );

    /**
     * Constructs a {@link SampleFollower} with the given drive subsystem.
     * @see SampleFollower for usage
     */
    public SampleFollower(Drive drive) {
        this.drive = drive;
    }

    /**
     * Follows the given SwerveSample by calculating the necessary chassis speeds
     * to track the sample's pose and commanding the drive subsystem accordingly.
     *
     * @param sample The SwerveSample to follow, containing the desired pose and feedforward velocities.
     */
    public void follow(SwerveSample sample) {
        // calculate chassis speeds to follow the sample
        Pose2d current = drive.getPose();
        Pose2d wants = sample.getPose();

        Logger.recordOutput("Auto/TargetPose", wants);

        double currentHeading = current.getRotation().getRadians();
        double targetHeading = wants.getRotation().getRadians();

        // generate speeds
        ChassisSpeeds cmd = new ChassisSpeeds(
            sample.vx + xCtl.calculate(current.getX(), wants.getX()),
            sample.vy + yCtl.calculate(current.getY(), wants.getY()),
            sample.omega + rotCtl.calculate(currentHeading, targetHeading)
        );

        // field-relative drive
        drive.drive(cmd);
    }
}
