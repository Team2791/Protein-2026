package frc.robot.subsystems.intake.pivot;

import frc.robot.util.SparkData;
import org.littletonrobotics.junction.AutoLog;

/**
 * Abstract IO class for the intake pivot mechanism.
 *
 * <p>Two Neo Vortex motors in leader/follower (inverted), driven to a
 * position setpoint via closed-loop PID.
 */
public abstract class PivotIO {

    /** Auto-logged data structure for pivot data */
    @AutoLog
    public static class PivotData {

        PivotData() {}

        /** Leader motor status */
        public SparkData leader = SparkData.empty();

        /** Follower motor status */
        public SparkData follower = SparkData.empty();
    }

    /** The current pivot data */
    public final PivotDataAutoLogged data = new PivotDataAutoLogged();

    /** Updates {@link #data} with the current pivot state */
    public abstract void update();

    /**
     * Sets the pivot position setpoint in radians (at the output shaft).
     *
     * @param position target position in rad
     */
    public abstract void setPosition(double position);
}
