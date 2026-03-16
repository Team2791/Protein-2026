package frc.robot.subsystems.climber.cylinder;

import frc.robot.data.RevPhData;
import org.littletonrobotics.junction.AutoLog;

/**
 * Abstract IO class for a single climber pneumatic cylinder.
 *
 * <p>
 * Each cylinder is driven by a single-acting {@link edu.wpi.first.wpilibj.Solenoid}
 * on the REV Pneumatics Hub. When engaged ({@code true}), the hook connects with
 * the climbing mechanism; when disengaged ({@code false}), it releases.
 *
 * <p>
 * The {@link Climber} subsystem manages three instances of this IO: one for
 * the inner hook and one each for the two outer hooks.
 */
public abstract class CylinderIO {

    /** Auto-logged data structure for a single cylinder */
    @AutoLog
    public static class CylinderData {

        CylinderData() {}

        /** Whether this cylinder is currently engaged (solenoid forward). */
        public boolean engaged = false;

        /** REV Pneumatics Hub status (shared across cylinders on the same hub). */
        public RevPhData hub = RevPhData.empty();
    }

    /** The current cylinder data */
    public final CylinderDataAutoLogged data = new CylinderDataAutoLogged();

    /** Updates {@link #data} with the current cylinder state */
    public abstract void update();

    /**
     * Sets the cylinder state.
     *
     * @param engaged {@code true} to engage (forward), {@code false} to disengage (reverse)
     */
    public abstract void set(boolean engaged);
}
