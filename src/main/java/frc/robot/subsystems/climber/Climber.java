package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.climber.axle.AxleIO;
import frc.robot.subsystems.climber.axle.AxleIO.AxleData;
import frc.robot.subsystems.climber.cylinder.CylinderIO;
import frc.robot.subsystems.climber.cylinder.CylinderIO.CylinderData;

/**
 * Climber subsystem controlling the axle rotation and pneumatic hook cylinders.
 *
 * <p>
 * This subsystem wraps an {@link AxleIO} (two SparkFlex motors on a shared
 * axle)
 * and three {@link CylinderIO} instances (inner hook, outer-left hook,
 * outer-right
 * hook) to provide a consistent interface regardless of whether running on real
 * hardware, in simulation, or in log replay.
 *
 * <p>
 * The climbing sequence alternates between expanded and contracted positions:
 *
 * <pre>
 *   kContract → kExpand → kContract = L1
 *   L1        → kExpand → kContract = L2
 *   L2        → kExpand → kContract = L3
 * </pre>
 *
 * <p>
 * In autonomous, the robot climbs only L1 using the inner hooks and
 * {@link #contractPartial()} instead of {@link #contract()}, which reaches L1
 * in a position where the robot can come back down.
 */
public class Climber extends SubsystemBase {

    /** Possible states of the axle rotation. */
    public enum State {
        /** Fully contracted (home position). */
        CONTRACT,
        /** Fully expanded. */
        EXPAND,
        /** Partially contracted (autonomous L1 — allows descent). */
        CONTRACT_PARTIAL,
    }

    /** The axle IO interface (two motors, position control). */
    final AxleIO axle;

    /** Inner hook cylinder. */
    final CylinderIO inner;

    /** Outer-left hook cylinder. */
    final CylinderIO outerLeft;

    /** Outer-right hook cylinder. */
    final CylinderIO outerRight;

    /** Current completed level (0 = ground, 1 = L1, etc.). */
    int level = 0;

    /** Current axle state. */
    State state = State.CONTRACT;

    /**
     * Constructs a Climber subsystem.
     *
     * @param axle       The axle IO implementation (motor control)
     * @param inner      The inner hook cylinder IO
     * @param outerLeft  The outer-left hook cylinder IO
     * @param outerRight The outer-right hook cylinder IO
     */
    public Climber(
        AxleIO axle,
        CylinderIO inner,
        CylinderIO outerLeft,
        CylinderIO outerRight
    ) {
        this.axle = axle;
        this.inner = inner;
        this.outerLeft = outerLeft;
        this.outerRight = outerRight;
    }

    /**
     * Returns a snapshot of the current axle motor data.
     *
     * @return A clone of the current {@link AxleData}
     */
    public AxleData axleData() {
        return axle.data.clone();
    }

    /**
     * Returns a snapshot of the inner cylinder data.
     *
     * @return A clone of the current inner {@link CylinderData}
     */
    public CylinderData innerData() {
        return inner.data.clone();
    }

    /**
     * Returns a snapshot of the outer-left cylinder data.
     *
     * @return A clone of the current outer-left {@link CylinderData}
     */
    public CylinderData outerLeftData() {
        return outerLeft.data.clone();
    }

    /**
     * Returns a snapshot of the outer-right cylinder data.
     *
     * @return A clone of the current outer-right {@link CylinderData}
     */
    public CylinderData outerRightData() {
        return outerRight.data.clone();
    }

    /**
     * Engages or disengages the inner hook cylinder.
     *
     * @param engaged {@code true} to engage, {@code false} to disengage
     */
    public void setInner(boolean engaged) {
        inner.set(engaged);
    }

    /**
     * Engages or disengages both outer hook cylinders simultaneously.
     *
     * @param engaged {@code true} to engage, {@code false} to disengage
     */
    public void setOuter(boolean engaged) {
        outerLeft.set(engaged);
        outerRight.set(engaged);
    }

    /**
     * Commands the axle to the expanded position.
     *
     * <p>
     * Sets the state to {@link State#EXPAND}.
     */
    public void expand() {
        state = State.EXPAND;
        axle.setPosition(ClimberConstants.kExpand);
    }

    /**
     * Commands the axle to the fully contracted position.
     *
     * <p>
     * If transitioning from {@link State#EXPAND}, increments the level
     * counter. Sets the state to {@link State#CONTRACT}.
     */
    public void contract() {
        if (state == State.EXPAND) {
            level++;
        }
        state = State.CONTRACT;
        axle.setPosition(ClimberConstants.kContract);
    }

    /**
     * Commands the axle to the partial contraction position.
     *
     * <p>
     * Used during autonomous to reach L1 in a position where the robot
     * can come back down. If transitioning from {@link State#EXPAND},
     * increments the level counter.
     */
    public void contractPartial() {
        if (state == State.EXPAND) {
            level++;
        }
        state = State.CONTRACT_PARTIAL;
        axle.setPosition(ClimberConstants.kContractPartial);
    }

    /**
     * Returns the current completed level.
     *
     * @return Level number (0 = ground, 1 = L1, 2 = L2, 3 = L3)
     */
    public int getLevel() {
        return level;
    }

    /**
     * Returns the current axle state.
     *
     * @return The current {@link State}
     */
    public State getState() {
        return state;
    }

    /** Updates all IO objects every robot loop. */
    @Override
    public void periodic() {
        axle.update();
        inner.update();
        outerLeft.update();
        outerRight.update();
    }
}
