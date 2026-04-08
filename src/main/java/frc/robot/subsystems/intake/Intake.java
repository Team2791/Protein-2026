package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.Roller.RollerState;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.roller.RollerIO;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

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

    /** Whether the intake is currently commanded to deploy. */
    boolean deployed = true;

    private boolean lastDeployed = false;
    private boolean pivotMoving = false;
    private boolean pivotDone = false;
    private double completedPosition = 0;

    /** Whether the intake should run IF DEPLOYED */
    RollerState roll = RollerState.kNormal;

    /**
     * Constructs an Intake subsystem.
     *
     * @param pivot  The pivot IO implementation
     * @param roller The roller IO implementation
     */
    public Intake(PivotIO pivot, RollerIO roller) {
        this.pivot = pivot;
        this.roller = roller;
        AutoLogOutputManager.addObject(this);
    }

    /** Returns the pivot IO data (for external reads). */
    public PivotIO.PivotData pivotData() {
        return pivot.data.clone();
    }

    /** Returns the roller IO data (for external reads). */
    public RollerIO.RollerData rollerData() {
        return roller.data.clone();
    }

    /** Checks conneciton statuses */
    @AutoLogOutput(key = "Intake/Ok")
    public boolean ok() {
        return pivot.data.leader.connected() && roller.data.leader.connected();
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

    public void roll(RollerState state) {
        this.roll = state;
    }

    @Override
    public void periodic() {
        pivot.update();
        roller.update();

        Logger.processInputs("Intake/Pivot", pivot.data);
        Logger.processInputs("Intake/Roller", roller.data);

        roller.set(roll.power);

        double vel = Math.abs(pivot.data.leader.velocity());

        if (deployed != lastDeployed) {
            pivotMoving = false;
            pivotDone = false;
            lastDeployed = deployed;
        }

        if (!pivotDone) {
            double power = deployed
                ? -IntakeConstants.Pivot.kDeployPower
                : IntakeConstants.Pivot.kDeployPower;
            pivot.set(power);

            if (
                !pivotMoving && vel > IntakeConstants.Pivot.kVelocityThreshold
            ) {
                pivotMoving = true;
            } else if (
                pivotMoving && vel <= IntakeConstants.Pivot.kVelocityThreshold
            ) {
                pivot.set(0);
                completedPosition = pivot.data.leader.position();
                pivotDone = true;
            }
        } else {
            double pos = pivot.data.leader.position();
            if (deployed && pos > completedPosition + IntakeConstants.Pivot.kPushThreshold) {
                // Ball pushing pivot up while deployed — push back down
                pivot.set(-IntakeConstants.Pivot.kDeployPower);
            } else if (!deployed && pos < completedPosition - IntakeConstants.Pivot.kPushThreshold) {
                // Something pushing pivot down while retracted — push back up
                pivot.set(IntakeConstants.Pivot.kDeployPower);
            } else {
                pivot.set(0);
            }
        }
    }
}
