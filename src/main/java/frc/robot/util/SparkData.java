package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Function;

/**
 * Data class representing the state of a Spark motor controller, including encoder readings,
 * electrical measurements, and connection status.
 *
 * <p>This class provides a structured way to capture and log relevant data from a Spark motor
 * controller for diagnostics and analysis.
 */
public record SparkData(
    /** The angular position of the motor in radians */
    double position,
    /** The angular velocity of the motor in radians per second */
    double velocity,
    /** The bus voltage of the motor in volts */
    double voltage,
    /** The output current of the motor in amps */
    double amps,
    /** The temperature of the motor in celsius */
    double temperature,
    /** The current [-1..1] output of the motor */
    double output,
    /** Whether the motor is connected */
    boolean connected
) implements Cloneable {
    /**
     * Reads the current state of a Spark motor controller and returns it as a SparkData instance.
     * @param spark The Spark motor controller to read from
     * @return A SparkData instance containing the current state of the motor controller
     */
    public static SparkData read(SparkBase spark) {
        return new SparkData(
            spark.getEncoder().getPosition(),
            spark.getEncoder().getVelocity(),
            spark.getBusVoltage(),
            spark.getOutputCurrent(),
            spark.getMotorTemperature(),
            spark.get(),
            spark.getLastError() == REVLibError.kOk
        );
    }

    /**
     * Reads the current state of a Spark motor controller, applying a conversion function to the
     * encoder readings, and returns it as a SparkData instance.
     * @param spark The Spark motor controller to read from
     * @param conversion Conversion from radians to desired position units (e.g. meters). Applied to both position and velocity.
     * @return A SparkData instance containing the converted state of the motor controller
     */
    public static SparkData read(
        SparkBase spark,
        Function<Double, Double> conversion
    ) {
        return new SparkData(
            conversion.apply(spark.getEncoder().getPosition()),
            conversion.apply(spark.getEncoder().getVelocity()),
            spark.getBusVoltage(),
            spark.getOutputCurrent(),
            spark.getMotorTemperature(),
            spark.get(),
            spark.getLastError() == REVLibError.kOk
        );
    }

    public static SparkData empty() {
        return new SparkData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
    }
}
