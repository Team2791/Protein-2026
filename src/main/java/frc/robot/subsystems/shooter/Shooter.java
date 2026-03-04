package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterData;

/**
 * Shooter subsystem for controlling the robot's ball-shooting mechanism.
 *
 * <p>This subsystem wraps a {@link ShooterIO} implementation to provide a consistent interface
 * for controlling the shooter regardless of whether running on real hardware, in simulation,
 * or in log replay.
 *
 * <p>Automatic speed control is handled by the IO layer based on robot position.
 * Manual override is available via {@link #setVelocity(double)} for debugging.
 */
public class Shooter extends SubsystemBase {

    /** The hardware/simulation IO interface for the shooter. */
    final ShooterIO io;

    /**
     * Constructs a Shooter subsystem with the given IO implementation.
     *
     * @param io The IO implementation to use (real hardware, replay, etc.)
     */
    public Shooter(ShooterIO io) {
        this.io = io;
    }

    /**
     * Returns a snapshot of the current shooter sensor data.
     *
     * @return A clone of the current {@link ShooterData}
     */
    public ShooterData data() {
        return io.data.clone();
    }

    /**
     * DEBUGGING: Sets the shooter wheel velocity directly, bypassing automatic control.
     *
     * @param velocity The target velocity in radians/second
     */
    public void setVelocity(double velocity) {
        io.setVelocity(velocity);
    }

    /** Calls {@link ShooterIO#update()} every robot loop to refresh sensor data. */
    @Override
    public void periodic() {
        io.update();
    }
}
