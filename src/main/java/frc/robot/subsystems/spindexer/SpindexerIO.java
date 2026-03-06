package frc.robot.subsystems.spindexer;

import frc.robot.util.SparkData;
import org.littletonrobotics.junction.AutoLog;

/**
 * Abstract IO class for the spindexer subsystem.
 *
 * <p>This class defines the interface for interacting with the spindexer hardware,
 * including a SparkFlex (Neo Vortex) and a SparkMax (Neo) motor, each run
 * independently at a fixed power via {@link #set(boolean)}.
 *
 * <p>Concrete implementations handle the specifics of communicating with the
 * underlying motor controllers.
 */
public abstract class SpindexerIO {

    /** Auto-logged data structure for spindexer data */
    @AutoLog
    public static class SpindexerData {

        SpindexerData() {}

        /** SparkFlex (Neo Vortex) motor status */
        public SparkData spindexer = SparkData.empty();

        /** SparkMax (Neo) motor status */
        public SparkData kicker = SparkData.empty();
    }

    /** The current spindexer data */
    public final SpindexerDataAutoLogged data = new SpindexerDataAutoLogged();

    /** Updates {@link #data} with the current spindexer state */
    public abstract void update();

    /**
     * Runs or stops both motors at their configured constant powers.
     *
     * @param running {@code true} to spin at the configured powers, {@code false} to stop
     */
    public abstract void set(boolean running);
}
