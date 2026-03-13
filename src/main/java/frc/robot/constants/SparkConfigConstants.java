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

            kSpindexer.encoder.positionConversionFactor(kTau);
            kSpindexer.encoder.velocityConversionFactor(kTau / 60.0);

            kKicker.encoder.positionConversionFactor(kTau);
            kKicker.encoder.velocityConversionFactor(kTau / 60.0);

            // idle mode
            kSpindexer.idleMode(SpindexerConstants.Motor.kIdleMode);
            kKicker.idleMode(SpindexerConstants.Motor.kIdleMode);
        }

        public static void apply(SparkFlex flex, SparkMax max) {
            flex.configure(kSpindexer, kReset, kPersist);
            max.configure(kKicker, kReset, kPersist);
        }
    }

    public static final class IntakeRoller {

        static final SparkFlexConfig kLeader;
        static final SparkFlexConfig kFollower;

        static {
            kLeader = new SparkFlexConfig();
            kFollower = new SparkFlexConfig();

            // current limits
            kLeader.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);
            kFollower.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);

            // encoder conversion: motor rad → wheel rad (includes reduction)
            kLeader.encoder.positionConversionFactor(
                kTau * IntakeConstants.Roller.kReduction
            );
            kLeader.encoder.velocityConversionFactor(
                (kTau * IntakeConstants.Roller.kReduction) / 60.0
            );

            // roller PID (velocity)
            ControlConstants.Intake.kRollerPid.register(kLeader);

            kLeader.idleMode(IntakeConstants.Roller.kIdleMode);
            kFollower.idleMode(IntakeConstants.Roller.kIdleMode);

            // leader-follower (inverted)
            kFollower.follow(
                IOConstants.Intake.kRollerLeader,
                IntakeConstants.Roller.kInvertFollower
            );
        }

        public static void apply(SparkFlex leader, SparkFlex follower) {
            leader.configure(kLeader, kReset, kPersist);
            follower.configure(kFollower, kReset, kPersist);

            ControlConstants.Intake.kRollerPid.register(leader);
        }
    }

    public static final class IntakePivot {

        static final SparkFlexConfig kLeader;
        static final SparkFlexConfig kFollower;

        static {
            kLeader = new SparkFlexConfig();
            kFollower = new SparkFlexConfig();

            // current limits
            kLeader.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);
            kFollower.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);

            // encoder conversion: motor rad → output rad (includes reduction)
            kLeader.encoder.positionConversionFactor(
                kTau * IntakeConstants.Pivot.kReduction
            );
            kLeader.encoder.velocityConversionFactor(
                (kTau * IntakeConstants.Pivot.kReduction) / 60.0
            );

            // pivot PID (position)
            ControlConstants.Intake.kPivotPid.register(kLeader);

            kLeader.idleMode(IntakeConstants.Pivot.kIdleMode);
            kFollower.idleMode(IntakeConstants.Pivot.kIdleMode);

            // leader-follower (inverted)
            kFollower.follow(
                IOConstants.Intake.kPivotLeader,
                IntakeConstants.Pivot.kInvertFollower
            );
        }

        public static void apply(SparkFlex leader, SparkFlex follower) {
            leader.configure(kLeader, kReset, kPersist);
            follower.configure(kFollower, kReset, kPersist);

            ControlConstants.Intake.kPivotPid.register(leader);
        }
    }

    public static final class Climber {

        static final SparkFlexConfig kLeader;
        static final SparkFlexConfig kFollower;

        static {
            kLeader = new SparkFlexConfig();
            kFollower = new SparkFlexConfig();

            // current limits
            kLeader.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);
            kFollower.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);

            // encoder conversion: motor rad → output rad (direct drive)
            kLeader.encoder.positionConversionFactor(
                kTau * ClimberConstants.kReduction
            );
            kLeader.encoder.velocityConversionFactor(
                (kTau * ClimberConstants.kReduction) / 60.0
            );

            // position PID
            ControlConstants.Climber.kPid.register(kLeader);

            kLeader.idleMode(ClimberConstants.Motor.kIdleMode);
            kFollower.idleMode(ClimberConstants.Motor.kIdleMode);

            // leader-follower
            kFollower.follow(
                IOConstants.Climber.kLeader,
                ClimberConstants.Motor.kInvertFollower
            );
        }

        public static void apply(SparkFlex leader, SparkFlex follower) {
            leader.configure(kLeader, kReset, kPersist);
            follower.configure(kFollower, kReset, kPersist);

            ControlConstants.Climber.kPid.register(leader);
        }
    }

    public static final ResetMode kReset = ResetMode.kResetSafeParameters;
    public static final PersistMode kPersist = PersistMode.kPersistParameters;
}
