package frc.robot.subsystems.spindexer;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.constants.SpindexerConstants;
import frc.robot.util.SparkData;

/**
 * Concrete {@link SpindexerIO} implementation for Spark motor controllers.
 *
 * <p>Uses a {@link SparkFlex} driving a Neo Vortex and a {@link SparkMax} driving
 * a standard Neo. Both motors are controlled independently at fixed duty-cycle
 * powers defined in {@link SpindexerConstants}.
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
        SparkConfigConstants.Spindexer.apply(spindexer, kicker);
    }

    @Override
    public void set(boolean running) {
        spindexer.set(running ? SpindexerConstants.kSpindexerPower : 0.0);
        kicker.set(running ? SpindexerConstants.kKickerPower : 0.0);
    }

    @Override
    public void update() {
        data.spindexer = SparkData.read(spindexer);
        data.kicker = SparkData.read(kicker);
    }
}
