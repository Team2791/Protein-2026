package frc.robot.constants;

import static frc.robot.util.MathPlus.kTau;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkConfigConstants {

    public static final class Shooter {

        static final SparkFlexConfig kLeader;
        static final SparkFlexConfig kFollower;

        static {
            kLeader = new SparkFlexConfig();
            kFollower = new SparkFlexConfig();

            // current limits
            kLeader.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);
            kFollower.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);

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

    public static final class Spindexer {

        static final SparkFlexConfig kSpindexer;
        static final SparkMaxConfig kKicker;

        static {
            kSpindexer = new SparkFlexConfig();
            kKicker = new SparkMaxConfig();

            // current limits
            kSpindexer.smartCurrentLimit(
                MotorConstants.NeoVortex.kCurrentLimit
            );
            kKicker.smartCurrentLimit(MotorConstants.Neo.kCurrentLimit);

            // idle mode
            kSpindexer.idleMode(SpindexerConstants.Motor.kIdleMode);
            kKicker.idleMode(SpindexerConstants.Motor.kIdleMode);
        }

        public static void apply(SparkFlex flex, SparkMax max) {
            flex.configure(kSpindexer, kReset, kPersist);
            max.configure(kKicker, kReset, kPersist);
        }
    }

    public static final ResetMode kReset = ResetMode.kResetSafeParameters;
    public static final PersistMode kPersist = PersistMode.kPersistParameters;
}
