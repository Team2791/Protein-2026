package frc.robot.subsystems.shooter;

/**
 * Replay implementation of {@link ShooterIO} for simulation/log replay.
 *
 * <p>This class is used when replaying recorded robot data from AdvantageKit logs.
 * It does not interact with any real hardware or simulation - all shooter data is
 * read directly from the logged {@link ShooterIO.ShooterData} values.
 *
 * <p>All methods are no-ops because replayed data is read-only.
 */
public class ShooterReplay extends ShooterIO {

    public ShooterReplay() {}

    /** No-op: velocity setpoint is not applied in replay mode. */
    @Override
    public void setVelocity(double velocity) {}

    /** No-op: data is populated from the log, not from hardware. */
    @Override
    public void update() {}
}
