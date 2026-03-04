package frc.robot.constants;

import static frc.robot.util.MathPlus.kTau;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

public class SparkConfigConstants {

    public static final class Shooter {

        static final SparkFlexConfig kLeader;
        static final SparkFlexConfig kFollower;

        static {
            kLeader = new SparkFlexConfig();
            kFollower = new SparkFlexConfig();

            // current limits
            kLeader.smartCurrentLimit(
                (int) MotorConstants.NeoVortex.kCurrentLimit
            );
            kFollower.smartCurrentLimit(
                (int) MotorConstants.NeoVortex.kCurrentLimit
            );

            // position and velocity factors
            kLeader.encoder.positionConversionFactor(kTau);
            kLeader.encoder.velocityConversionFactor(kTau / 60.0);

            // pid constants
            ControlConstants.Shooter.kPid.register(kLeader);

            // idle mode
            kLeader.idleMode(ShooterConstants.Motor.kIdleMode);
            kFollower.idleMode(ShooterConstants.Motor.kIdleMode);

            // leader-follower
            kFollower.follow(
                IOConstants.Shooter.kLeader,
                ShooterConstants.Motor.kInvertFollower
            );
        }

        public static void apply(SparkFlex leader, SparkFlex follower) {
            leader.configure(kLeader, kReset, kPersist);
            follower.configure(kFollower, kReset, kPersist);

            ControlConstants.Shooter.kPid.register(leader);
        }
    }

    public static final ResetMode kReset = ResetMode.kResetSafeParameters;
    public static final PersistMode kPersist = PersistMode.kPersistParameters;
}
