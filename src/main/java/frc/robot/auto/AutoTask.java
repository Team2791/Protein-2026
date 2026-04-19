package frc.robot.auto;

import static frc.robot.auto.generated.ChoreoVars.Poses.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.pathfind.Pathfind;
import frc.robot.commands.drive.pathfind.ResetPose;
import frc.robot.commands.shooter.PointAndShoot;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import java.util.function.Function;

/**
 * Enumeration of discrete autonomous routine steps.
 *
 * <p>Each constant represents a single logical unit of work the robot can perform
 * during autonomous — a starting-position reset, an intake drive, a scoring sequence,
 * etc. Steps are composed at runtime by {@link AutoSelector} into a full autonomous
 * {@link Command} based on the driver's dashboard selections.
 *
 * <h3>How steps are defined</h3>
 * Every constant is constructed with a {@link Function}&lt;{@link Subsystems}, {@link Command}&gt;
 * that builds the actual WPILib command for this step given the three live subsystems.
 * The function is called lazily by {@link #full} each time an auto routine is built,
 * so subsystem references are never captured at enum-init time.
 *
 * <p>Starting-position steps (e.g. {@link #StartPos1}) additionally supply a {@link #resetTo}
 * pose that {@link AutoSelector#build} uses to teleport the robot before the command runs,
 * ensuring odometry is correct regardless of where the robot was placed on the field.
 *
 * <h3>Navigation</h3>
 * All drive-to steps use {@link Pathfind#targetLock}, which holds the target heading
 * during the entire transit and snaps to it again at the end. All poses are expressed in
 * blue-alliance coordinates; {@link frc.robot.util.AllianceUtil} handles the red-alliance
 * flip transparently.
 *
 * <h3>Extending</h3>
 * Add a new constant with a lambda that builds its command, and a matching {@code case}
 * in {@link #label()} for dashboard display. No other changes are needed — {@link AutoSelector}
 * automatically includes every constant in the chooser.
 *
 * @see AutoSelector
 * @see Pathfind
 */
public enum AutoTask {
    // -------------------------------------------------------------------------
    // Starting positions — each resets odometry to the named pose
    // -------------------------------------------------------------------------

    /** Teleports odometry to starting position 1 (left side of the field). */
    StartPos1(s -> new ResetPose(s.drive(), pos_1)),

    /** Teleports odometry to starting position 2 (center of the field). */
    StartPos2(s -> new ResetPose(s.drive(), pos_2)),

    /** Teleports odometry to starting position 3 (right side of the field). */
    StartPos3(s -> new ResetPose(s.drive(), pos_3)),

    // -------------------------------------------------------------------------
    // Center ball collection — intake from the center line
    // -------------------------------------------------------------------------

    /**
     * Drives to the right-side center ball cluster, then moves to the center-right
     * collection point at reduced speed.
     */
    CenterBallsRhs(s ->
        Commands.sequence(
            new Pathfind(s.drive(), balls_rhs, pos_3.getRotation()),
            Pathfind.targetLock(s.drive(), center_rhs, 0.75)
        )
    ),

    /**
     * Drives to the left-side center ball cluster, then moves to the center-left
     * collection point at reduced speed.
     */
    CenterBallsLhs(s ->
        Commands.sequence(
            new Pathfind(s.drive(), balls_lhs, pos_1.getRotation()),
            Pathfind.targetLock(s.drive(), center_lhs, 0.75)
        )
    ),

    // -------------------------------------------------------------------------
    // Scoring sequences — each starts the shooter, drives to a scoring position,
    // waits for the shooter to reach setpoint, then shoots
    // -------------------------------------------------------------------------

    /**
     * Scores from the right trench position: spins up shooter, drives to
     * {@code trench_score}, waits for speed, then calls {@link PointAndShoot}.
     */
    TrenchScoreRhs(s ->
        Commands.sequence(
            new SetShooter(s.shooter(), ShooterConstants.Setpoint.kAuto),
            Pathfind.targetLock(s.drive(), trench_score),
            Commands.waitUntil(s.shooter()::inTolerance),
            new PointAndShoot(s.drive(), s.spindexer(), s.shooter())
        )
    ),

    /**
     * Scores from the left trench position: spins up shooter, drives to
     * {@code pos_1}, waits for speed, then calls {@link PointAndShoot}.
     */
    TrenchScoreLhs(s ->
        Commands.sequence(
            new SetShooter(s.shooter(), ShooterConstants.Setpoint.kAuto),
            Pathfind.targetLock(s.drive(), pos_1),
            Commands.waitUntil(s.shooter()::inTolerance),
            new PointAndShoot(s.drive(), s.spindexer(), s.shooter())
        )
    ),

    /**
     * Shuttle (lob) shot from the right side: spins up to high setpoint, drives to
     * {@code shuttle_rhs}, waits for speed, then shoots for up to 3 seconds.
     */
    ShuttleRhs(s ->
        Commands.sequence(
            new SetShooter(s.shooter(), ShooterConstants.Setpoint.kHigh),
            Pathfind.targetLock(s.drive(), shuttle_rhs),
            Commands.waitUntil(s.shooter()::inTolerance),
            new Shoot(s.spindexer()).withTimeout(3)
        )
    ),

    /**
     * Shuttle (lob) shot from the left side: spins up to high setpoint, drives to
     * {@code shuttle_lhs}, waits for speed, then shoots for up to 3 seconds.
     */
    ShuttleLhs(s ->
        Commands.sequence(
            new SetShooter(s.shooter(), ShooterConstants.Setpoint.kHigh),
            Pathfind.targetLock(s.drive(), shuttle_lhs),
            Commands.waitUntil(s.shooter()::inTolerance),
            new Shoot(s.spindexer()).withTimeout(3)
        )
    ),

    // -------------------------------------------------------------------------
    // Intake positioning — drive to an intake spot without shooting
    // -------------------------------------------------------------------------

    /** Drives to the outpost intake position. */
    Outpost(s -> Pathfind.targetLock(s.drive(), outpost)),

    /** Drives to the depot intake position. */
    Depot(s -> Pathfind.targetLock(s.drive(), depot)),

    // -------------------------------------------------------------------------
    // Other scoring targets
    // -------------------------------------------------------------------------

    /**
     * Scores from the outpost scoring position: spins up shooter, drives to
     * {@code outpost_score}, waits for speed, then calls {@link PointAndShoot}.
     */
    OutpostScore(s ->
        Commands.sequence(
            new SetShooter(s.shooter(), ShooterConstants.Setpoint.kAuto),
            Pathfind.targetLock(s.drive(), outpost_score),
            Commands.waitUntil(s.shooter()::inTolerance),
            new PointAndShoot(s.drive(), s.spindexer(), s.shooter())
        )
    ),

    /**
     * Scores from the hub scoring position: spins up shooter, drives to
     * {@code hub_score}, waits for speed, then calls {@link PointAndShoot}.
     */
    HubScore(s ->
        Commands.sequence(
            new SetShooter(s.shooter(), ShooterConstants.Setpoint.kAuto),
            Pathfind.targetLock(s.drive(), hub_score),
            Commands.waitUntil(s.shooter()::inTolerance),
            new PointAndShoot(s.drive(), s.spindexer(), s.shooter())
        )
    ),

    // -------------------------------------------------------------------------
    // Terminal step — signals AutoSelector to stop building further commands
    // -------------------------------------------------------------------------

    /**
     * No-op terminal step. When {@link AutoSelector} encounters this in the selection
     * list it stops appending commands. Also used as the default "do nothing" option.
     */
    Cancel(s -> new WaitCommand(0));

    // -------------------------------------------------------------------------
    // Enum infrastructure
    // -------------------------------------------------------------------------

    /**
     * Bundle of live subsystem references passed to the step's {@link #perform} function.
     *
     * <p>Passed as a record rather than three separate parameters so that the
     * {@link Function} signatures stay short and new subsystems can be added without
     * touching every constant's lambda.
     */
    public record Subsystems(
        Drive drive,
        Shooter shooter,
        Spindexer spindexer
    ) {}

    /**
     * Lazily-evaluated command factory for this step.
     * Called by {@link #full} when an autonomous routine is assembled.
     */
    final Function<Subsystems, Command> perform;

    AutoTask(Function<Subsystems, Command> f) {
        this.perform = f;
    }

    /**
     * Returns a human-readable label for this step, used by
     * {@link AutoSelector} to populate dashboard choosers.
     */
    String label() {
        return switch (this) {
            case StartPos1 -> "Position 1 (Left)";
            case StartPos2 -> "Position 2 (Middle)";
            case StartPos3 -> "Position 3 (Right)";
            case CenterBallsLhs -> "Center Intake (Left)";
            case CenterBallsRhs -> "Center Intake (Right)";
            case TrenchScoreLhs -> "Trench Score (Left)";
            case TrenchScoreRhs -> "Trench Score (Right)";
            case ShuttleLhs -> "Shuttle (Left)";
            case ShuttleRhs -> "Shuttle (Right)";
            case Outpost -> "Outpost";
            case OutpostScore -> "Outpost Score";
            case HubScore -> "Position 2 Score";
            case Depot -> "Depot";
            case Cancel -> "All Done";
        };
    }

    /**
     * Builds and returns the WPILib {@link Command} for this step.
     *
     * @param drive     drive subsystem
     * @param shooter   shooter subsystem
     * @param spindexer spindexer subsystem
     * @return the command that executes this autonomous step
     */
    Command full(Drive drive, Shooter shooter, Spindexer spindexer) {
        return perform.apply(new Subsystems(drive, shooter, spindexer));
    }
}
