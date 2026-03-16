package frc.robot.subsystems.climber.axle;

import frc.robot.data.SparkData;
import org.littletonrobotics.junction.AutoLog;

/**
 * Abstract IO class for the climber axle mechanism.
 *
 * <p>
 * Two SparkFlex (Neo Vortex) motors on a shared axle in leader/follower,
 * driven to a position setpoint via closed-loop PID. The axle controls the
 * rotation of the climbing mechanism between contracted and expanded states.
 */
public abstract class AxleIO {

    /** Auto-logged data structure for axle data */
    @AutoLog
    public static class AxleData {

        AxleData() {}

        /** Leader motor status */
        public SparkData leader = SparkData.empty();

        /** Follower motor status */
        public SparkData follower = SparkData.empty();
    }

    /** The current axle data */
    public final AxleDataAutoLogged data = new AxleDataAutoLogged();

    /** Updates {@link #data} with the current axle state */
    public abstract void update();

    /**
     * Sets the axle position setpoint in radians (at the output shaft).
     *
     * @param position target position in rad
     */
    public abstract void setPosition(double position);
}
