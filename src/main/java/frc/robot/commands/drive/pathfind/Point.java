package frc.robot.commands.drive.pathfind;

import static frc.robot.util.MathPlus.kTau;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drive.Drive;

/**
 * Command for nearby pose-targeted navigation using PIDs.
 *
 * <p>
 * This command is used as a helper for {@link Pathfind}. It is
 * used as the final step: close to the target, switch to PID control.
 *
 * <p>
 * Navigate uses a {@link HolonomicDriveController} with three independent PID controllers
 * to move the robot to a target pose. The controller calculates the required
 * chassis speeds (x, y, rotation) to reach the target.
 */
public class Point extends Command {

    static class TunableController extends PIDController {

        static final TunableController instance = new TunableController();

        private TunableController() {
            super(
                ControlConstants.Nearby.kTurnP,
                ControlConstants.Nearby.kTurnI,
                ControlConstants.Nearby.kTurnD
            );
            super.enableContinuousInput(0, kTau);
            super.setTolerance(
                ControlConstants.Nearby.kTolerance.getRotation().getRadians()
            );

            SmartDashboard.putData("PID/Point", this);
        }
    }

    /** Get the supplier on {@link #initialize()} */
    final Rotation2d target;

    /** High-level holonomic controller combining all three PID loops. */
    final PIDController ctl = TunableController.instance;

    /** Drive subsystem for motion control. */
    final Drive drive;

    /**
     * Constructs a Nearby command.
     *
     * <p>
     * Initializes the HolonomicDriveController with the three PID controllers,
     * enables continuous input for rotation (angles wrap around), and sets the
     * position tolerance for determining when the target is reached.
     *
     * @param drive Drive subsystem for movement control
     * @param target The target supplier
     */
    public Point(Drive drive, Rotation2d target) {
        this.drive = drive;
        this.target = target;

        addRequirements(drive);
    }

    /**
     * Command execution: Calculate and send chassis speeds to reach target.
     *
     * <p>
     * Uses the {@link HolonomicDriveController} to calculate the required
     * chassis speeds (vx, vy, ω) based on current pose, target pose, and
     * controller gains.
     */
    @Override
    public final void execute() {
        Pose2d robot = drive.getPose();
        // Calculate speeds using holonomic controller, with a target velocity of 1 cm/s (0.01 m/s)
        double power = ctl.calculate(
            robot.getRotation().getRadians(),
            target.getRadians()
        );
        // Send robot-relative speeds to drive
        drive.drive(new ChassisSpeeds(0, 0, power));
    }

    /**
     * Command end: Stop robot and clean up target visualization.
     *
     * <p>
     * Called when the command finishes (either by {@link #isFinished()} or interruption).
     * Removes the target from the field visualization and stops the robot.
     *
     * @param interrupted true if command was interrupted, false if finished normally
     */
    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drive.runVelocity(new ChassisSpeeds());
    }

    /**
     * Checks if the robot has reached the target pose.
     *
     * <p>
     * Uses the {@link HolonomicDriveController}'s tolerance checking to determine if
     * both position and rotation are within acceptable thresholds.
     *
     * @return true if at target within tolerance, false otherwise
     */
    @Override
    public boolean isFinished() {
        return ctl.atSetpoint();
    }
}
