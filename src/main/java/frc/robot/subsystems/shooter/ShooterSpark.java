package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.util.SparkData;

/**
 * Concrete {@link ShooterIO} implementation for Spark motor controllers.
 */
public class ShooterSpark extends ShooterIO {

    /** The leader SparkFlex motor controller */
    final SparkFlex leader = new SparkFlex(
        IOConstants.Shooter.kLeader,
        MotorType.kBrushless
    );

    /** The follower SparkFlex motor controller */
    final SparkFlex follower = new SparkFlex(
        IOConstants.Shooter.kFollower,
        MotorType.kBrushless
    );

    /** The controller for the shooter */
    final SparkClosedLoopController controller =
        leader.getClosedLoopController();

    public ShooterSpark() {
        SparkConfigConstants.Shooter.apply(leader, follower);
    }

    @Override
    public void setVelocity(double velocity) {
        controller.setSetpoint(velocity, ControlType.kVelocity);
    }

    @Override
    public void update() {
        data.leader = SparkData.read(leader);
        data.follower = SparkData.read(follower);
    }
}
