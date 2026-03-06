package frc.robot.subsystems.intake.pivot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.util.SparkData;

/**
 * Concrete {@link PivotIO} implementation for Spark motor controllers.
 *
 * <p>Uses two SparkFlex/Neo Vortex motors in leader-follower with
 * position closed-loop on the leader.
 */
public class PivotSpark extends PivotIO {

    /** Leader SparkFlex for the pivot */
    final SparkFlex leader = new SparkFlex(
        IOConstants.Intake.kPivotLeader,
        MotorType.kBrushless
    );

    /** Follower SparkFlex for the pivot */
    final SparkFlex follower = new SparkFlex(
        IOConstants.Intake.kPivotFollower,
        MotorType.kBrushless
    );

    /** Closed-loop controller for pivot position */
    final SparkClosedLoopController controller =
        leader.getClosedLoopController();

    public PivotSpark() {
        SparkConfigConstants.IntakePivot.apply(leader, follower);
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
