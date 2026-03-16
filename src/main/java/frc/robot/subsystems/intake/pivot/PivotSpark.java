package frc.robot.subsystems.intake.pivot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.data.SparkData;

/**
 * Concrete {@link PivotIO} implementation for Spark motor controllers.
 *
 * <p>
 * Uses two SparkFlex/Neo Vortex motors in leader-follower with
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
        leader.configure(
            SparkConfigConstants.IntakePivot.kLeader,
            SparkConfigConstants.kReset,
            SparkConfigConstants.kPersist
        );
        follower.configure(
            SparkConfigConstants.IntakePivot.kFollower,
            SparkConfigConstants.kReset,
            SparkConfigConstants.kPersist
        );

        leader.getEncoder().setPosition(0);
    }

    @Override
    public void setPosition(double position) {
        controller.setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public void resetPosition(double position) {
        leader.getEncoder().setPosition(position);
    }

    @Override
    public void update() {
        data.leader = SparkData.read(leader);
        data.follower = SparkData.read(follower);
    }
}
