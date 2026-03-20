package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.data.SparkData;

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
        leader.configure(
            SparkConfigConstants.Shooter.kLeader,
            SparkConfigConstants.kReset,
            SparkConfigConstants.kPersist
        );
        follower.configure(
            SparkConfigConstants.Shooter.kFollower,
            SparkConfigConstants.kReset,
            SparkConfigConstants.kPersist
        );
    }

    @Override
    public void setVelocity(double velocity) {
        controller.setSetpoint(velocity, ControlType.kVelocity);
    }

    @Override
    public void update() {
        data.leader = SparkData.read(leader, data.leader);
        data.follower = SparkData.read(follower, data.follower);
    }
}
