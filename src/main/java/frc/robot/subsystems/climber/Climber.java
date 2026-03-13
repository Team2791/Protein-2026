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
 * axle) and three {@link CylinderIO} instances (inner hook, outer-left hook,
 * outer-right hook) to provide a consistent interface regardless of whether
 * running on real hardware, in simulation, or in log replay.
 *
 * <p>
 * The climbing sequence alternates between expanded and contracted positions:
 *
 * <pre>
 *   CONTRACT → EXPAND → CONTRACT = L1
 *   L1       → EXPAND → CONTRACT = L2
 *   L2       → EXPAND → CONTRACT = L3
 * </pre>
 *
 * <p>
 * In autonomous, the robot climbs only L1 using the inner hooks and
 * {@link ClimberConstants.Position#CONTRACT_PARTIAL} instead of
 * {@link ClimberConstants.Position#CONTRACT}, which reaches L1
 * in a position where the robot can come back down.
 */
public class Climber extends SubsystemBase {

    /** The axle IO interface (two motors, position control). */
    final AxleIO axle;

    /** Inner hook cylinder. */
    final CylinderIO inner;

    /** Outer-left hook cylinder. */
    final CylinderIO outerLeft;

    /** Outer-right hook cylinder. */
    final CylinderIO outerRight;

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
     * Commands the axle to the given position setpoint.
     *
     * @param position The target {@link ClimberConstants.Position}
     */
    public void setPosition(ClimberConstants.Position position) {
        axle.setPosition(position.radians);
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
