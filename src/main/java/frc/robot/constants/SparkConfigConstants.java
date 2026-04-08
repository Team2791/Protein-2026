package frc.robot.constants;

import static frc.robot.util.MathPlus.kTau;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkConfigConstants {

    public static final class Shooter {

        public static final SparkFlexConfig kLeader;
        public static final SparkFlexConfig kFollower;

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
            kLeader.closedLoop.pid(
                ControlConstants.Shooter.kP,
                ControlConstants.Shooter.kI,
                ControlConstants.Shooter.kD
            );
            kLeader.closedLoop.feedForward.sv(
                ControlConstants.Shooter.kShooterS,
                ControlConstants.Shooter.kShooterV
            );
            kLeader.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            // idle mode
            kLeader.idleMode(ShooterConstants.Motor.kIdleMode);
            kFollower.idleMode(ShooterConstants.Motor.kIdleMode);

            // leader-follower
            kFollower.follow(
                IOConstants.Shooter.kLeader,
                ShooterConstants.Motor.kInvertFollower
            );
        }
    }

    public static final class Spindexer {

        public static final SparkFlexConfig kSpindexer;
        public static final SparkMaxConfig kKicker;

        static {
            kSpindexer = new SparkFlexConfig();
            kKicker = new SparkMaxConfig();

            // current limits
            kSpindexer.smartCurrentLimit(
                MotorConstants.NeoVortex.kCurrentLimit
            );
            kKicker.smartCurrentLimit(50);

            kSpindexer.encoder.positionConversionFactor(kTau);
            kSpindexer.encoder.velocityConversionFactor(kTau / 60.0);

            kKicker.encoder.positionConversionFactor(kTau);
            kKicker.encoder.velocityConversionFactor(kTau / 60.0);

            // idle mode
            kSpindexer.idleMode(SpindexerConstants.Motor.kIdleMode);
            kKicker.idleMode(SpindexerConstants.Motor.kIdleMode);
        }
    }

    public static final class IntakeRoller {

        public static final SparkFlexConfig kLeader;
        public static final SparkFlexConfig kFollower;

        static {
            kLeader = new SparkFlexConfig();
            kFollower = new SparkFlexConfig();

            // current limits
            kLeader.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);
            kFollower.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);

            // encoder conversion: motor rad → wheel rad (includes reduction)
            kLeader.encoder.positionConversionFactor(
                kTau / IntakeConstants.Roller.kReduction
            );
            kLeader.encoder.velocityConversionFactor(
                kTau / 60.0 / IntakeConstants.Roller.kReduction
            );

            kLeader.idleMode(IntakeConstants.Roller.kIdleMode);
            kFollower.idleMode(IntakeConstants.Roller.kIdleMode);

            // leader-follower (inverted)
            kFollower.follow(
                IOConstants.Intake.kRollerLeader,
                IntakeConstants.Roller.kInvertFollower
            );
        }
    }

    public static final class IntakePivot {

        public static final SparkFlexConfig kLeader;
        public static final SparkFlexConfig kFollower;

        static {
            kLeader = new SparkFlexConfig();
            kFollower = new SparkFlexConfig();

            // current limits
            kLeader.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);
            kFollower.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);

            // encoder conversion: motor rev and rpm → output rad and rad/sec
            kLeader.encoder.positionConversionFactor(
                kTau / IntakeConstants.Pivot.kReduction
            );
            kLeader.encoder.velocityConversionFactor(
                kTau / 60.0 / IntakeConstants.Pivot.kReduction
            );

            // pivot PID (position)
            kLeader.closedLoop.pid(
                ControlConstants.Intake.kPivotP,
                ControlConstants.Intake.kPivotI,
                ControlConstants.Intake.kPivotD
            );
            kLeader.closedLoop.feedForward
                .kS(ControlConstants.Intake.kPivotS)
                .kCos(ControlConstants.Intake.kPivotG)
                .kCosRatio(1.0 / kTau);
            kLeader.closedLoop.allowedClosedLoopError(
                IntakeConstants.Pivot.kTolerance,
                ClosedLoopSlot.kSlot0
            );
            kLeader.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            kLeader.idleMode(IntakeConstants.Pivot.kIdleMode);
            kFollower.idleMode(IntakeConstants.Pivot.kIdleMode);

            // leader-follower (inverted)
            kFollower.follow(
                IOConstants.Intake.kPivotLeader,
                IntakeConstants.Pivot.kInvertFollower
            );
        }
    }

    public static final class Climber {

        public static final SparkFlexConfig kLeader;
        public static final SparkFlexConfig kFollower;

        static {
            kLeader = new SparkFlexConfig();
            kFollower = new SparkFlexConfig();

            // current limits
            kLeader.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);
            kFollower.smartCurrentLimit(MotorConstants.NeoVortex.kCurrentLimit);

            // encoder conversion: motor rad → output rad (direct drive)
            kLeader.encoder.positionConversionFactor(
                kTau / ClimberConstants.kReduction
            );
            kLeader.encoder.velocityConversionFactor(
                kTau / 60.0 / ClimberConstants.kReduction
            );

            // position PID
            kLeader.closedLoop.pid(
                ControlConstants.Climber.kP,
                ControlConstants.Climber.kI,
                ControlConstants.Climber.kD
            );
            kLeader.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            kLeader.idleMode(ClimberConstants.Motor.kIdleMode);
            kFollower.idleMode(ClimberConstants.Motor.kIdleMode);

            // leader-follower
            kFollower.follow(
                IOConstants.Climber.kLeader,
                ClimberConstants.Motor.kInvertFollower
            );
        }
    }

    public static final ResetMode kReset = ResetMode.kResetSafeParameters;
    public static final PersistMode kPersist = PersistMode.kPersistParameters;
}
