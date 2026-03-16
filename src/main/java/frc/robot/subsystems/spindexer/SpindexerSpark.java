package frc.robot.subsystems.spindexer;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.data.SparkData;

/**
 * Concrete {@link SpindexerIO} implementation for Spark motor controllers.
 *
 * <p>
 * Uses a {@link SparkFlex} driving a Neo Vortex and a {@link SparkMax} driving
 * a standard Neo. Both motors are controlled independently at fixed duty-cycle
 * powers.
 */
public class SpindexerSpark extends SpindexerIO {

    /** SparkFlex driving the Neo Vortex motor */
    final SparkFlex spindexer = new SparkFlex(
        IOConstants.Spindexer.kSpindexer,
        MotorType.kBrushless
    );

    /** SparkMax driving the standard Neo motor */
    final SparkMax kicker = new SparkMax(
        IOConstants.Spindexer.kKicker,
        MotorType.kBrushless
    );

    public SpindexerSpark() {
        spindexer.configure(
            SparkConfigConstants.Spindexer.kSpindexer,
            SparkConfigConstants.kReset,
            SparkConfigConstants.kPersist
        );
        kicker.configure(
            SparkConfigConstants.Spindexer.kKicker,
            SparkConfigConstants.kReset,
            SparkConfigConstants.kPersist
        );
    }

    @Override
    public void setSpindexer(double power) {
        spindexer.set(power);
    }

    @Override
    public void setKicker(double power) {
        kicker.set(power);
    }

    @Override
    public void update() {
        data.spindexer = SparkData.read(spindexer);
        data.kicker = SparkData.read(kicker);
    }
}
