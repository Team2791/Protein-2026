package frc.robot.subsystems.climber.cylinder;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.data.RevPhData;

/**
 * Concrete {@link CylinderIO} implementation for the REV Pneumatics Hub.
 *
 * <p>
 * Manages a single {@link Solenoid} and reads diagnostics from the shared
 * {@link PneumaticHub}. Multiple instances share a reference to the same hub
 * for data reads.
 */
public class CylinderPH extends CylinderIO {

    /** The Solenoid controlling this cylinder */
    final Solenoid solenoid;

    /** Shared reference to the REV Pneumatics Hub for diagnostic reads */
    final PneumaticHub hub;

    /**
     * Constructs a CylinderPH for a single solenoid on the REV PH.
     *
     * @param hub     The shared PneumaticHub instance
     * @param channel The solenoid channel
     */
    public CylinderPH(PneumaticHub hub, int channel) {
        this.hub = hub;
        this.solenoid = hub.makeSolenoid(channel);
    }

    @Override
    public void set(boolean engaged) {
        solenoid.set(engaged);
    }

    @Override
    public void update() {
        data.engaged = solenoid.get();
        data.hub = RevPhData.read(hub);
    }
}
