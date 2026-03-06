package frc.robot.subsystems.intake.roller;

import frc.robot.util.SparkData;
import org.littletonrobotics.junction.AutoLog;

/**
 * Abstract IO class for the intake roller mechanism.
 *
 * <p>Two Neo Vortex motors in leader/follower (inverted), driven at a
 * velocity setpoint to match drivetrain ground speed.
 */
public abstract class RollerIO {

    /** Auto-logged data structure for roller data */
    @AutoLog
    public static class RollerData {

        RollerData() {}

        /** Leader motor status */
        public SparkData leader = SparkData.empty();

        /** Follower motor status */
        public SparkData follower = SparkData.empty();
    }

    /** The current roller data */
    public final RollerDataAutoLogged data = new RollerDataAutoLogged();

    /** Updates {@link #data} with the current roller state */
    public abstract void update();

    /**
     * Sets the roller velocity in radians/second (at the wheel).
     *
     * @param velocity target velocity in rad/s
     */
    public abstract void setVelocity(double velocity);
}
