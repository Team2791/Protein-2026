package frc.robot.subsystems.shooter;

import frc.robot.data.SparkData;
import org.littletonrobotics.junction.AutoLog;

/**
 * Abstract IO class for the shooter subsystem.
 *
 * <p>
 * This class defines the interface for interacting with the shooter hardware,
 * including methods for setting shooter speeds and reading sensor data.
 *
 * <p>
 * Concrete implementations of this class should handle the specifics of
 * communicating with the shooter hardware, such as motor controllers and encoders.
 */
public abstract class ShooterIO {

    /** Auto-logged data structure for shooter data */
    @AutoLog
    public static class ShooterData {

        ShooterData() {}

        /** Leader status */
        public SparkData leader = SparkData.empty();

        /** Follower status */
        public SparkData follower = SparkData.empty();
    }

    /** The current shooter data */
    public final ShooterDataAutoLogged data = new ShooterDataAutoLogged();

    /** Updates {@link #data} with the current shooter state */
    public abstract void update();

    /** Sets the velocity in radians/second */
    public abstract void setVelocity(double velocity);
}
