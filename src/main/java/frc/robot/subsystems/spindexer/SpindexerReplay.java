package frc.robot.subsystems.spindexer;

/**
 * Replay implementation of {@link SpindexerIO} for simulation/log replay.
 *
 * <p>This class is used when replaying recorded robot data from AdvantageKit logs.
 * It does not interact with any real hardware or simulation — all spindexer data is
 * read directly from the logged {@link SpindexerIO.SpindexerData} values.
 *
 * <p>All methods are no-ops because replayed data is read-only.
 */
public class SpindexerReplay extends SpindexerIO {

    public SpindexerReplay() {}

    /** No-op: motor output is not applied in replay mode. */
    @Override
    public void set(boolean running) {}

    /** No-op: data is populated from the log, not from hardware. */
    @Override
    public void update() {}
}
