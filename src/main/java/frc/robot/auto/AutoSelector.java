package frc.robot.auto;

import choreo.auto.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.auto.generated.ChoreoTraj;
import frc.robot.commands.pathfind.Pathfind;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.*;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * AutoSelector provides a three-stage autonomous routine selection system for FRC robots.
 *
 * This class creates a hierarchical selection interface using three dashboard choosers that
 * dynamically update based on previous selections. The first chooser allows selection of
 * starting positions, the second chooser populates with valid transitions from the first
 * selection, and the third chooser populates with valid transitions from the second selection.
 *
 * The selector uses an AutoGraph to determine valid transitions between AutoNodes and
 * automatically loads all available Choreo trajectories into the factory cache for
 * efficient path planning.
 *
 * Key Features:
 * - Dynamic chooser population based on graph transitions
 * - Automatic trajectory caching from ChoreoTraj
 * - Support for canceling autonomous sequences at any stage
 * - Integration with Elastic notifications for error reporting
 * - Alliance-aware pose handling with automatic field flipping
 *
 * Usage:
 * 1. Create an AutoSelector instance with a Drive subsystem
 * 2. Call build() to generate an AutoRoutine based on current selections
 * 3. The routine will execute the selected path with appropriate transitions
 *
 * The selector handles null selections gracefully and provides feedback through
 * Elastic notifications when trajectory transitions are not available.
 *
 * @see AutoFactory
 * @see AutoNode
 * @see AutoGraph
 * @see ChoreoTraj
 */
public class AutoSelector {

    final AutoFactory factory;
    final SampleFollower follower;

    LoggedDashboardChooser<AutoNode> s1;
    LoggedDashboardChooser<AutoNode> s2;
    LoggedDashboardChooser<AutoNode> s3;

    /**
     * Constructs an AutoSelector with the given Drive subsystem.
     * @param drive Drive subsystem
     * @see AutoSelector
     */
    public AutoSelector(Drive drive) {
        follower = new SampleFollower(drive);
        factory = new AutoFactory(
            drive::getPose,
            drive::setPose,
            follower::follow,
            true,
            drive,
            (traj, starting) -> {
                // this function is called if a trajectory starts or finished
                // the `starting` parameter is true if the trajectory is starting, false if it is ending
                // if we are ending a trajectory, do cleanup...
                if (!starting) {
                    Pose2d[] empty = new Pose2d[0];
                    Logger.recordOutput("Auto/Trajectory", empty);
                    drive.field.getObject("Auto/Trajectory").setPoses(empty);
                    return;
                }

                // otherwise, log+visualize the trajectory
                Pose2d[] trj = traj.getPoses();
                Pose2d[] flipped = AllianceUtil.autoflip(trj).orElse(trj);
                Logger.recordOutput("Auto/Trajectory", flipped);
                drive.field.getObject("Auto/Trajectory").setPoses(flipped);
            }
        );

        for (ChoreoTraj traj : ChoreoTraj.ALL_TRAJECTORIES.values()) {
            factory.cache().loadTrajectory(traj.name());
        }

        s1 = chooser("Autonomous Selector 1");
        s2 = chooser("Autonomous Selector 2");
        s3 = chooser("Autonomous Selector 3");

        add(s1, AutoNode.POS1);
        add(s1, AutoNode.POS2);
        add(s1, AutoNode.POS3);

        s1.onChange(this::updateS2);
    }

    /**
     * Helper method to create a LoggedDashboardChooser for AutoNode selections.
     * @param name Name of the chooser for dashboard display
     * @return A new LoggedDashboardChooser instance
     */
    private LoggedDashboardChooser<AutoNode> chooser(String name) {
        return new LoggedDashboardChooser<>(name);
    }

    /**
     * Helper method to add an AutoNode option to a chooser.
     * @param c The chooser to add the option to
     * @param n The AutoNode to add as an option
     */
    private void add(LoggedDashboardChooser<AutoNode> c, AutoNode n) {
        c.addOption(n.label(), n);
    }

    /**
     * Populates the second chooser based on the first selection and sets up the next stage of selection.
     *
     * <p>When a selection is made in the first chooser, this method retrieves all valid transitions
     * from the selected AutoNode using the AutoGraph and populates the second chooser with those options.
     * It also adds a "Cancel" option to allow users to stop at the first selection if desired.
     *
     * @param chooser The chooser to populate with options
     * @param from The AutoNode from which to find valid transitions
     */
    private void populate(
        LoggedDashboardChooser<AutoNode> chooser,
        AutoNode from
    ) {
        AutoGraph.TRANSITIONS.getOrDefault(from, Map.of())
            .keySet()
            .forEach(n -> add(chooser, n));

        add(chooser, AutoNode.CANCEL);
    }

    /**
     * Updates the second chooser based on the selection from the first chooser and sets up the next stage of selection.
     * @param sel The AutoNode selected in the first chooser
     */
    private void updateS2(AutoNode sel) {
        s2.getSendableChooser().close();
        s2 = chooser("Autonomous Selector 2");
        s2.onChange(this::updateS3);
        populate(s2, sel);
    }

    /**
     * Updates the third chooser based on the selection from the second chooser and sets up the next stage of selection.
     * @param sel The AutoNode selected in the second chooser
     */
    private void updateS3(AutoNode sel) {
        s3.getSendableChooser().close();
        s3 = chooser("Autonomous Selector 3");
        populate(s3, sel);
    }

    /**
     * Builds an AutoRoutine based on the current selections in the choosers.
     *
     * <p>This method retrieves the selected AutoNodes from the three choosers and constructs a command sequence
     * that follows the selected path through the AutoGraph. It handles null selections and "Cancel" options
     * gracefully, allowing for flexible autonomous routine construction.
     *
     * @param drive The Drive subsystem to use for trajectory following
     * @return An AutoRoutine representing the selected autonomous path, or null if no valid selection was made
     */
    public AutoRoutine build(Drive drive) {
        AutoRoutine routine = factory.newRoutine("Auto");

        AutoNode a = s1.get();
        AutoNode b = s2.get();
        AutoNode c = s3.get();

        if (a == null || b == null) return null;
        if (b == AutoNode.CANCEL) {
            Command seq = a.onEnter(routine);
            if (seq != null) routine.active().onTrue(seq);
            return routine;
        }
        if (c == null) return null;

        List<AutoNode> path = List.of(a, b, c)
            .stream()
            .filter(n -> n != null && n != AutoNode.CANCEL)
            .toList();

        Command cmd = a.onEnter(routine);

        for (int i = 0; i < path.size() - 1; i++) {
            AutoNode from = path.get(i);
            AutoNode to = path.get(i + 1);

            ChoreoTraj traj = AutoGraph.TRANSITIONS.get(from).get(to);
            if (traj == null) {
                Elastic.sendNotification(
                    new Notification(
                        NotificationLevel.ERROR,
                        "Failed to find trajectory",
                        String.format("From %s to %s", from.label(), to.label())
                    )
                );
                return routine;
            }

            cmd = Commands.sequence(
                cmd == null ? Commands.none() : cmd,
                new Pathfind.Supplied(drive, () -> {
                    // SAFETY: choosing from dashboard implies we have DS connection
                    return AllianceUtil.unsafe.autoflip(traj.initialPoseBlue());
                }),
                traj.asAutoTraj(routine).cmd(),
                to.onEnter(routine)
            );
        }

        if (cmd != null) routine.active().onTrue(cmd);
        return routine;
    }
}
