package frc.robot.subsystems.intake.roller;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.data.SparkData;

/**
 * Concrete {@link RollerIO} implementation for Spark motor controllers.
 *
 * <p>
 * Uses two SparkFlex/Neo Vortex motors in leader-follower with
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

    public RollerSpark() {
        leader.configure(
            SparkConfigConstants.IntakeRoller.kLeader,
            SparkConfigConstants.kReset,
            SparkConfigConstants.kPersist
        );
        follower.configure(
            SparkConfigConstants.IntakeRoller.kFollower,
            SparkConfigConstants.kReset,
            SparkConfigConstants.kPersist
        );
    }

    @Override
    public void set(double output) {
        leader.set(output);
    }

    @Override
    public void update() {
        data.leader = SparkData.read(leader, data.leader);
        data.follower = SparkData.read(follower, data.follower);
    }
}
