package frc.robot.subsystems.intake.pivot;

/**
 * Replay implementation of {@link PivotIO} for simulation/log replay.
 *
 * <p>
 * All methods are no-ops because replayed data is read-only.
 */
public class PivotReplay extends PivotIO {

    public PivotReplay() {}

    /** No-op: position setpoint is not applied in replay mode. */
    @Override
    public void setPosition(double position) {}

    /** No-op: data is populated from the log, not from hardware. */
    @Override
    public void update() {}
}
