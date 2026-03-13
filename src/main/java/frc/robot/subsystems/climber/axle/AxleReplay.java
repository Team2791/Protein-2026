package frc.robot.subsystems.climber.axle;

/**
 * Replay implementation of {@link AxleIO} for simulation/log replay.
 *
 * <p>
 * This class is used when replaying recorded robot data from AdvantageKit logs.
 * It does not interact with any real hardware or simulation — all axle data is
 * read directly from the logged {@link AxleIO.AxleData} values.
 *
 * <p>
 * All methods are no-ops because replayed data is read-only.
 */
public class AxleReplay extends AxleIO {

    public AxleReplay() {}

    /** No-op: position setpoint is not applied in replay mode. */
    @Override
    public void setPosition(double position) {}

    /** No-op: data is populated from the log, not from hardware. */
    @Override
    public void update() {}
}
