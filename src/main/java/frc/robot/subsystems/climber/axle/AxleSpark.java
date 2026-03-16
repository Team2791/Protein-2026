package frc.robot.subsystems.climber.axle;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.data.SparkData;

/**
 * Concrete {@link AxleIO} implementation for Spark motor controllers.
 *
 * <p>
 * Uses two SparkFlex/Neo Vortex motors in leader-follower with
 * position closed-loop on the leader.
 */
public class AxleSpark extends AxleIO {

    /** Leader SparkFlex for the axle */
    final SparkFlex leader = new SparkFlex(
        IOConstants.Climber.kLeader,
        MotorType.kBrushless
    );

    /** Follower SparkFlex for the axle */
    final SparkFlex follower = new SparkFlex(
        IOConstants.Climber.kFollower,
        MotorType.kBrushless
    );

    /** Closed-loop controller for axle position */
    final SparkClosedLoopController controller =
        leader.getClosedLoopController();

    public AxleSpark() {
        leader.configure(
            SparkConfigConstants.Climber.kLeader,
            SparkConfigConstants.kReset,
            SparkConfigConstants.kPersist
        );
        follower.configure(
            SparkConfigConstants.Climber.kFollower,
            SparkConfigConstants.kReset,
            SparkConfigConstants.kPersist
        );
    }

    @Override
    public void setPosition(double position) {
        controller.setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public void update() {
        data.leader = SparkData.read(leader);
        data.follower = SparkData.read(follower);
    }
}
