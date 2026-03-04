package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.GameConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil;
import frc.robot.util.SparkData;
import frc.robot.util.Vec2;

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

    /** The drive subsystem */
    final Drive drive;

    /**
     * When {@code true}, the shooter is in manual mode (velocity set via {@link #setVelocity}).
     * In manual mode, the automatic distance-based control in {@link #update()} is bypassed.
     */
    boolean manual = false;

    public ShooterSpark(Drive drive) {
        this.drive = drive;

        SparkConfigConstants.Shooter.apply(leader, follower);
    }

    @Override
    public void setVelocity(double velocity) {
        manual = true;
        controller.setSetpoint(velocity, ControlType.kVelocity);
    }

    @Override
    public void update() {
        data.leader = SparkData.read(leader);
        data.follower = SparkData.read(follower);

        if (manual) return;

        // Automatic control logic
        Pose2d pose = drive.getPose();
        Pose2d blue = AllianceUtil.autoflip(pose).orElse(pose);

        if (blue.getX() > ShooterConstants.kSpinUpThreshold) {
            controller.setSetpoint(0, ControlType.kVelocity);
            return;
        }

        Vec2 hub = new Vec2(GameConstants.Objects.kHub);
        Vec2 delta = hub.sub(new Vec2(blue));
        double dist = delta.mag();

        // regress
        double vel = ShooterConstants.Regression.apply(dist);
        controller.setSetpoint(vel, ControlType.kVelocity);
    }
}
