package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.util.Vec2;

/**
 * Intake subsystem controlling roller and pivot mechanisms.
 *
 * <p>
 * This subsystem wraps separate {@link PivotIO} and {@link RollerIO}
 * implementations to provide a consistent interface regardless of whether
 * running on real hardware, in simulation, or in log replay.
 *
 * <p>
 * Calling {@link #deploy(boolean) deploy(true)} commands the pivot to its deployed
 * position and, once the pivot is within tolerance, spins the rollers at a speed
 * matched to the drivetrain. Calling {@code deploy(false)} stops the rollers
 * immediately and retracts the pivot.
 */
public class Intake extends SubsystemBase {

    /** The pivot IO interface. */
    final PivotIO pivot;

    /** The roller IO interface. */
    final RollerIO roller;

    /** The drive subsystem, used to read current chassis speed. */
    final Drive drive;

    /** Whether the intake is currently commanded to deploy. */
    boolean deployed = false;

    /** The intake should initally deploy once at the beginning of the match */
    boolean autoDeployed = false;

    /**
     * Constructs an Intake subsystem.
     *
     * @param pivot  The pivot IO implementation
     * @param roller The roller IO implementation
     * @param drive  The drive subsystem for speed-matching
     */
    public Intake(PivotIO pivot, RollerIO roller, Drive drive) {
        this.pivot = pivot;
        this.roller = roller;
        this.drive = drive;
    }

    /** Returns the pivot IO data (for external reads). */
    public PivotIO.PivotData pivotData() {
        return pivot.data.clone();
    }

    /** Returns the roller IO data (for external reads). */
    public RollerIO.RollerData rollerData() {
        return roller.data.clone();
    }

    /**
     * Deploys or retracts the intake.
     *
     * <p>
     * When deployed, the pivot moves to the down setpoint and the rollers
     * begin spinning (matched to drivetrain speed) once the pivot reaches
     * position. When retracted, the rollers stop immediately and the pivot
     * returns to 0 rad.
     *
     * @param deployed {@code true} to deploy, {@code false} to retract
     */
    public void deploy(boolean deployed) {
        this.deployed = deployed;
    }

    @Override
    public void periodic() {
        if (!autoDeployed && !deployed && DriverStation.isEnabled()) {
            deploy(true);
            autoDeployed = true;
        }

        pivot.update();
        roller.update();

        if (!deployed) {
            roller.setVelocity(0);
            pivot.setPosition(0.0);
            return;
        }

        // Command the pivot to the deployed position
        pivot.setPosition(IntakeConstants.Pivot.kDeployedPosition);

        if (pivot.data.leader.pid().atTarget()) {
            roller.setVelocity(0);
            return;
        }

        // Compute drivetrain/robot-relative velocity (m/s)
        Vec2 vel = new Vec2(drive.getRobotSpeeds());

        // shooter is +X, intake is -Y
        // match all intake-directed velocity
        double yVel = Math.max(0, -vel.y);

        // Convert linear speed to wheel angular velocity (rad/s).
        // The Spark encoder conversion factor already accounts for the
        // gear reduction, so we only need to divide by wheel radius.
        // v = r * omega -> omega = v / r
        double omega = yVel / IntakeConstants.Roller.kWheelRadius;

        roller.setVelocity(omega);
    }
}
