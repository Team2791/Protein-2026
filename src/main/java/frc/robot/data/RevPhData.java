package frc.robot.data;

import edu.wpi.first.wpilibj.PneumaticHub;

/**
 * Data class representing the state of a REV Pneumatics Hub, including
 * pressure readings, compressor status, and solenoid channel states.
 *
 * <p>
 * This class provides a structured way to capture and log relevant data
 * from a REV Pneumatics Hub for diagnostics and analysis, analogous to
 * {@link SparkData} for motor controllers.
 */
public record RevPhData(
    /** Pressure on analog channel 0 in PSI */
    double pressure,
    /** Whether the compressor is currently compressing */
    boolean compressing,
    /** Compressor current draw in amps */
    double compressorCurrent,
    /** Bitmask of solenoid channel states (LSB = channel 0) */
    int solenoids,
    /** Input voltage to the PH module in volts */
    double voltage,
    /** Total current draw of the PH module in amps */
    double current
) implements Cloneable {
    /**
     * Reads the current state of a REV Pneumatics Hub.
     *
     * @param ph The PneumaticHub to read from
     * @return A RevPhData instance containing the current state
     */
    public static RevPhData read(PneumaticHub ph) {
        return new RevPhData(
            ph.getPressure(0),
            ph.getCompressor(),
            ph.getCompressorCurrent(),
            ph.getSolenoids(),
            ph.getInputVoltage(),
            ph.getSolenoidsTotalCurrent()
        );
    }

    /**
     * Returns an empty RevPhData instance with default values.
     *
     * @return A RevPhData with all values zeroed/false
     */
    public static RevPhData empty() {
        return new RevPhData(0.0, false, 0.0, 0, 0.0, 0.0);
    }

    @Override
    public RevPhData clone() {
        return new RevPhData(
            pressure,
            compressing,
            compressorCurrent,
            solenoids,
            voltage,
            current
        );
    }
}
