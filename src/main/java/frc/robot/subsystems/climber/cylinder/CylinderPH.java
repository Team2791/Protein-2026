package frc.robot.subsystems.climber.cylinder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.util.RevPhData;

/**
 * Concrete {@link CylinderIO} implementation for the REV Pneumatics Hub.
 *
 * <p>
 * Manages a single {@link DoubleSolenoid} and reads diagnostics from the
 * shared {@link PneumaticHub}. Multiple instances share a reference to the
 * same hub for data reads.
 */
public class CylinderPH extends CylinderIO {

    /** The DoubleSolenoid controlling this cylinder */
    final DoubleSolenoid solenoid;

    /** Shared reference to the REV Pneumatics Hub for diagnostic reads */
    final PneumaticHub hub;

    /**
     * Constructs a CylinderPH for a single solenoid on the REV PH.
     *
     * @param hub        The shared PneumaticHub instance
     * @param fwdChannel The forward solenoid channel (engage)
     * @param revChannel The reverse solenoid channel (disengage)
     */
    public CylinderPH(PneumaticHub hub, int fwdChannel, int revChannel) {
        this.hub = hub;
        this.solenoid = new DoubleSolenoid(
            hub.getModuleNumber(),
            PneumaticsModuleType.REVPH,
            fwdChannel,
            revChannel
        );
    }

    @Override
    public void set(boolean engaged) {
        solenoid.set(engaged ? Value.kForward : Value.kReverse);
    }

    @Override
    public void update() {
        data.engaged = solenoid.get() == Value.kForward;
        data.hub = RevPhData.read(hub);
    }
}
