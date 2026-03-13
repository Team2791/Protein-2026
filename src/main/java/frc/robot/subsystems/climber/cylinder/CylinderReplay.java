package frc.robot.subsystems.climber.cylinder;

/**
 * Replay implementation of {@link CylinderIO} for simulation/log replay.
 *
 * <p>
 * This class is used when replaying recorded robot data from AdvantageKit logs.
 * It does not interact with any real hardware or simulation — all cylinder data
 * is
 * read directly from the logged {@link CylinderIO.CylinderData} values.
 *
 * <p>
 * All methods are no-ops because replayed data is read-only.
 */
public class CylinderReplay extends CylinderIO {

    public CylinderReplay() {}

    /** No-op: solenoid output is not applied in replay mode. */
    @Override
    public void set(boolean engaged) {}

    /** No-op: data is populated from the log, not from hardware. */
    @Override
    public void update() {}
}
