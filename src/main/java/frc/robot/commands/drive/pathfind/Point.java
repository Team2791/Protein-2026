package frc.robot.commands.drive.pathfind;

import static frc.robot.util.MathPlus.kTau;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil;

/**
 * Rotates the robot to a target heading using a PID controller.
 *
 * <p>Uses a single {@link TunableController} singleton so that SmartDashboard tuning
 * applies to every instance. The controller has continuous input enabled over
 * {@code [0, 2π]} to ensure the shortest-arc rotation path is always taken.
 *
 * <p>Heading is measured in blue-alliance coordinates via
 * {@link AllianceUtil.unsafe#autoflip}, so {@code target} should always be expressed
 * as a blue-frame angle.
 *
 * <p>This command requires {@link Drive} and cannot run in parallel with {@link PIDLerp}.
 * Use {@link LockedPIDLerp} when simultaneous translation and a locked heading are needed.
 *
 * @see LockedPIDLerp
 * @see Pathfind
 */
public class Point extends Command {

    /**
     * Singleton PID controller for heading control, published to SmartDashboard as
     * {@code PID/Point}.
     *
     * <p>A singleton is used because only one Point command can run at a time (Drive is
     * required), so sharing the instance is safe and avoids duplicate SmartDashboard keys.
     * Continuous input is enabled over {@code [0, 2π]} so the controller always rotates
     * via the shorter arc.
     */
    static class TunableController extends PIDController {

        static final TunableController instance = new TunableController();

        private TunableController() {
            super(
                ControlConstants.PIDLerp.kTurnP,
                ControlConstants.PIDLerp.kTurnI,
                ControlConstants.PIDLerp.kTurnD
            );
            super.enableContinuousInput(0, kTau);
            super.setTolerance(ControlConstants.PIDLerp.kRotationTolerance);

            SmartDashboard.putData("PID/Point", this);
        }
    }

    /** Blue-alliance target heading. */
    final Rotation2d target;

    /**
     * Shared PID controller. Reset in {@link #initialize()} each time this command starts
     * to prevent integral windup carried over from a previous run.
     */
    final PIDController ctl = TunableController.instance;

    /** Drive subsystem for motion control. */
    final Drive drive;

    /**
     * Creates a Point command that rotates to {@code target} and requires {@link Drive}.
     *
     * @param drive  drive subsystem
     * @param target target heading in blue-alliance coordinates
     */
    public Point(Drive drive, Rotation2d target) {
        this.drive = drive;
        this.target = target;

        addRequirements(drive);
    }

    /**
     * Resets the PID controller to clear accumulated integral and derivative state
     * from any previous run of this singleton instance.
     */
    @Override
    public void initialize() {
        ctl.reset();
    }

    /**
     * Computes the rotation output toward {@link #target} using the current
     * blue-frame heading, without sending any commands to the drive.
     *
     * <p>Package-private so that {@link LockedPIDLerp} can call this and inject the
     * result into {@link PIDLerp#execute(double)}, combining translation and rotation
     * into a single {@code drive.drive()} call without scheduling this as a separate command.
     *
     * @return rotation speed in rad/s
     */
    double calculate() {
        Pose2d blue = AllianceUtil.unsafe.autoflip(drive.getPose());
        return ctl.calculate(
            blue.getRotation().getRadians(),
            target.getRadians()
        );
    }

    /**
     * Commands zero translation and applies the rotation output from {@link #calculate()}.
     */
    @Override
    public final void execute() {
        double power = calculate();
        drive.drive(new ChassisSpeeds(0, 0, power));
    }

    /** Stops the drive. */
    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds());
    }

    /**
     * Finishes when the heading error is within {@link ControlConstants.PIDLerp#kRotationTolerance}
     * and angular velocity has decayed below 0.3 rad/s.
     */
    @Override
    public boolean isFinished() {
        return (
            ctl.atSetpoint() &&
            drive.getChassisSpeeds().omegaRadiansPerSecond < 0.3
        );
    }
}
