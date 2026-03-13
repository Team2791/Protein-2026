package frc.robot.util;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.SparkConfigConstants;
import java.util.ArrayList;
import java.util.List;

/**
 * Wraps a WPILib {@link PIDController} to allow live PID tuning via Shuffleboard/SmartDashboard
 * that is automatically applied to a REV Spark motor controller's closed-loop slot.
 *
 * <p>
 * Workflow:
 * <ol>
 *   <li>Construct with initial gains and a dashboard name.
 *   <li>Call {@link #register(SparkBaseConfig)} with the Spark's config object to bake in the current gains.
 *   <li>Call {@link #register(SparkBase)} to bind the motor that will be reconfigured on change.
 *   <li>Call {@link #updateAll()} each loop; if gains changed on the dashboard, the Spark is reconfigured.
 * </ol>
 */
public class TunableSparkPID {

    /**
     * Snapshot of PID gains used to detect changes without floating-point surprises.
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    record PIDConstants(double kP, double kI, double kD) {
        public boolean equals(PIDConstants other) {
            return kP == other.kP && kI == other.kI && kD == other.kD;
        }
    }

    /** A list of instances to update every loop. */
    static final List<TunableSparkPID> instances = new ArrayList<>();

    /** The WPILib PIDController exposed to SmartDashboard for live tuning. */
    final PIDController inner;

    /** The last-applied PID gains snapshot, used to detect dashboard changes. */
    PIDConstants old;

    /** The Spark configuration object whose closed-loop slot is updated when gains change. */
    SparkBaseConfig config;

    /** The Spark motor controller to reconfigure when gains change. */
    SparkBase motor;

    /**
     * Constructs a TunableSparkPID with initial PID gains and registers it on SmartDashboard.
     *
     * @param kP Initial proportional gain
     * @param kI Initial integral gain
     * @param kD Initial derivative gain
     * @param name Dashboard key suffix (published under {@code "PID/<name>"})
     */
    public TunableSparkPID(double kP, double kI, double kD, String name) {
        inner = new PIDController(kP, kI, kD);
        SmartDashboard.putData("PID/" + name, inner);

        old = collect();
    }

    /**
     * Writes the current gains into the provided Spark config's closed-loop slot
     * and stores the config for later reconfiguration via {@link #updateAll()}.
     *
     * <p>
     * Call this before applying the config to the motor.
     *
     * @param config The Spark config object to write PID gains into
     */
    public void register(SparkBaseConfig config) {
        PIDConstants current = collect();

        config.closedLoop.pid(current.kP, current.kI, current.kD);
        this.config = config;
    }

    /**
     * Binds the Spark motor controller that will be reconfigured when gains change.
     *
     * @param motor The Spark motor controller to reconfigure on gain change
     */
    public void register(SparkBase motor) {
        this.motor = motor;
        instances.add(this);
    }

    /**
     * Collects the current PID gains from the dashboard-backed {@link PIDController}.
     *
     * @return A snapshot of the current P, I, D values
     */
    PIDConstants collect() {
        return new PIDConstants(inner.getP(), inner.getI(), inner.getD());
    }

    /**
     * Checks whether PID gains have changed on the dashboard and, if so, applies
     * the new gains to the Spark motor controller in real time.
     *
     * <p>
     * Call this every robot loop iteration.
     */
    public static void updateAll() {
        for (TunableSparkPID that : instances) {
            PIDConstants current = that.collect();
            if (current.equals(that.old)) continue; // no change since last update, skip

            that.config.closedLoop.pid(current.kP, current.kI, current.kD);
            that.motor.configure(
                that.config,
                SparkConfigConstants.kReset,
                SparkConfigConstants.kPersist
            );
        }
    }
}
