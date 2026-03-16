package frc.robot.subsystems.intake.roller;

/**
 * Replay implementation of {@link RollerIO} for simulation/log replay.
 *
 * <p>
 * All methods are no-ops because replayed data is read-only.
 */
public class RollerReplay extends RollerIO {

    public RollerReplay() {}

    /** No-op: velocity setpoint is not applied in replay mode. */
    @Override
    public void set(double velocity) {}

    /** No-op: data is populated from the log, not from hardware. */
    @Override
    public void update() {}
}
