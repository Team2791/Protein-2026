package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.generated.ChoreoVars;
import frc.robot.commands.drive.pathfind.PIDLerp;
import frc.robot.commands.drive.pathfind.Point;
import frc.robot.commands.shooter.PointAndShoot;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.constants.ShooterConstants.Setpoint;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.AllianceUtil;

/**
 * Enumeration representing different autonomous routine nodes/states for the robot.
 * Each node defines a specific action or sequence of actions that the robot should
 * perform during autonomous mode, such as shooting, intaking, moving to positions,
 * or climbing.
 */
public enum AutoNode {
    POS1(ChoreoVars.Poses.pos_1),
    POS2(ChoreoVars.Poses.pos_2),
    POS3(ChoreoVars.Poses.pos_3),
    BALLS_LHS(ChoreoVars.Poses.balls_lhs),
    BALLS_RHS(ChoreoVars.Poses.balls_rhs),
    CENTER_LHS(ChoreoVars.Poses.center_lhs, 1),
    CENTER_RHS(ChoreoVars.Poses.center_rhs, 1),
    TRENCH_SCORE(ChoreoVars.Poses.trench_score),
    HUB_SCORE(ChoreoVars.Poses.hub_score),
    OUTPOST(ChoreoVars.Poses.outpost),
    OUTPOST_SCORE(ChoreoVars.Poses.outpost_score),
    CANCEL(new Pose2d());

    final Pose2d pose;
    final double maxVel;

    AutoNode(Pose2d pose) {
        this(pose, DriveConstants.maxSpeedMetersPerSec);
    }

    AutoNode(Pose2d pose, double maxVel) {
        this.pose = pose;
        this.maxVel = maxVel;
    }

    /**
     * Returns a human-readable label for this autonomous node.
     * Used for display purposes in dashboards or selection interfaces.
     *
     * @return A descriptive string label for this node
     */
    String label() {
        return switch (this) {
            case POS1 -> "Position 1 (Left)";
            case POS2 -> "Position 2 (Middle)";
            case POS3 -> "Position 3 (Right)";
            case BALLS_LHS -> "Center Intake (Left)";
            case BALLS_RHS -> "Center Intake (Right)";
            case CENTER_LHS -> "Center (Left)";
            case CENTER_RHS -> "Center (Right)";
            case TRENCH_SCORE -> "Trench Score";
            case HUB_SCORE -> "Center Score";
            case OUTPOST -> "Outpost";
            case OUTPOST_SCORE -> "Outpost Score";
            case CANCEL -> "All Done";
        };
    }

    /**
     * Defines the command sequence that should be executed when entering this autonomous node.
     * Creates and returns the appropriate command based on the node type, which may include
     * shooting sequences, intake operations, trajectory following, aiming, and climbing.
     *
     * @param routine The autonomous routine context that provides access to trajectory commands
     * @return The command to execute when entering this node, or null if no action is required
     */
    Command onEnter(Drive drive, Shooter shooter, Spindexer spindexer) {
        return switch (this) {
            case POS1, POS3 -> Commands.none();
            case POS2 -> new SetShooter(shooter, Setpoint.kMedium);
            case BALLS_LHS, BALLS_RHS -> Commands.none(); // I/A in future, start intake
            case CENTER_LHS, CENTER_RHS -> new SetShooter(
                shooter,
                Setpoint.kMedium
            );
            case TRENCH_SCORE, HUB_SCORE -> new Shoot(spindexer);
            case OUTPOST -> new WaitCommand(5); // Wait load balls
            case OUTPOST_SCORE -> new PointAndShoot(drive, spindexer, null);
            case CANCEL -> Commands.none();
        };
    }

    Command full(Drive drive, Shooter shooter, Spindexer spindexer) {
        int deg = (int) Math.round(pose.getRotation().getDegrees());

        Command point = new Point(
            drive,
            AllianceUtil.unsafe.autoflip(pose.getRotation())
        );
        Command lerp1 = new PIDLerp(drive, pose, maxVel);
        Command enter = onEnter(drive, shooter, spindexer);

        if (deg == 0 || deg == 180) {
            return new SequentialCommandGroup(lerp1, point, enter);
        }

        return new SequentialCommandGroup(
            point,
            lerp1,
            new Point(drive, AllianceUtil.unsafe.autoflip(pose.getRotation())),
            enter
        );
    }
}
