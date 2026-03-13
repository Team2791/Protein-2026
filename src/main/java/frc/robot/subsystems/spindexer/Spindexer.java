package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SpindexerConstants;
import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerData;

/**
 * Spindexer subsystem controlling two independent brushless motors.
 *
 * <p>
 * This subsystem wraps a {@link SpindexerIO} implementation to provide a
 * consistent interface regardless of whether running on real hardware, in
 * simulation, or in log replay.
 *
 * <p>
 * Both motors are controlled together via {@link #set(boolean)}: when
 * {@code true} each motor runs at its configured constant power; when
 * {@code false} both stop.
 */
public class Spindexer extends SubsystemBase {

    /** The hardware/simulation IO interface for the spindexer. */
    final SpindexerIO io;

    /**
     * Constructs a Spindexer subsystem with the given IO implementation.
     *
     * @param io The IO implementation to use (real hardware, replay, etc.)
     */
    public Spindexer(SpindexerIO io) {
        this.io = io;
    }

    /**
     * Returns a snapshot of the current spindexer sensor data.
     *
     * @return A clone of the current {@link SpindexerData}
     */
    public SpindexerData data() {
        return io.data.clone();
    }

    /**
     * Runs or stops the spindexer motors at their configured constant powers.
     *
     * @param running {@code true} to spin at configured powers, {@code false} to stop
     */
    public void set(boolean running) {
        io.setSpindexer(running ? SpindexerConstants.kSpindexerPower : 0.0);
        io.setKicker(running ? SpindexerConstants.kKickerPower : 0.0);
    }

    /** Calls {@link SpindexerIO#update()} every robot loop to refresh sensor data. */
    @Override
    public void periodic() {
        io.update();
    }
}
