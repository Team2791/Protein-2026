package frc.robot.subsystems.intake.roller;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.util.SparkData;

/**
 * Concrete {@link RollerIO} implementation for Spark motor controllers.
 *
 * <p>Uses two SparkFlex/Neo Vortex motors in leader-follower with
 * velocity closed-loop on the leader.
 */
public class RollerSpark extends RollerIO {

    /** Leader SparkFlex for the rollers */
    final SparkFlex leader = new SparkFlex(
        IOConstants.Intake.kRollerLeader,
        MotorType.kBrushless
    );

    /** Follower SparkFlex for the rollers */
    final SparkFlex follower = new SparkFlex(
        IOConstants.Intake.kRollerFollower,
        MotorType.kBrushless
    );

    /** Closed-loop controller for roller velocity */
    final SparkClosedLoopController controller =
        leader.getClosedLoopController();

    public RollerSpark() {
        SparkConfigConstants.IntakeRoller.apply(leader, follower);
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
