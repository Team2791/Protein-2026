package frc.robot.auto;

import choreo.auto.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.auto.generated.ChoreoTraj;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.AllianceUtil;
import java.util.*;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Builds autonomous routines from an ordered sequence of {@link AutoTask} steps
 * selected via a cascade of up to {@value #NUM_CHOOSERS} SmartDashboard choosers.
 *
 * <h3>Selection model</h3>
 * <p>The first chooser ({@code Auto/StartPos}) always offers the three starting positions
 * plus {@link AutoTask#Cancel}. When the driver picks a value, {@link #update(int, AutoTask)}
 * fires, populates chooser 1 ({@code Auto/Task1}) with all available tasks, and registers
 * the same listener on it. Each subsequent chooser cascades the same way: selecting a value
 * populates the next chooser. Selecting {@link AutoTask#Cancel} at any stage leaves
 * subsequent choosers empty and signals {@link #build} to stop there.
 *
 * <p>If the driver goes back and changes an earlier chooser, {@link #cutoff(int)} tears down
 * all later choosers and rebuilds them from scratch so stale selections cannot leak into
 * the built routine.
 *
 * <h3>Building a routine</h3>
 * <p>Call {@link #build(Drive, Shooter, Spindexer)} from {@code Robot.autonomousInit()}.
 * The method iterates {@link #current}, prepends an odometry reset for starting-position
 * steps, and sequences the {@link AutoTask#full} commands into a single {@link Command}.
 *
 * <h3>Trajectory caching</h3>
 * <p>All {@link ChoreoTraj} trajectories are loaded into the {@link AutoFactory} cache at
 * construction time so that the first autonomous run incurs no I/O delay.
 *
 * @see AutoTask
 */
public class AutoSelector {

    /** Choreo trajectory factory used for path-following segments. */
    final AutoFactory factory;

    /**
     * Ordered list of the driver's current selections, one entry per chooser that has
     * been touched. {@link AutoTask#Cancel} is not stored here — it acts as a sentinel
     * during {@link #build} and terminates iteration.
     */
    final List<AutoTask> current = new ArrayList<>();

    /**
     * Live list of dashboard choosers. Index {@code i} corresponds to the
     * {@code (i+1)}-th decision the driver makes (index 0 = start position).
     */
    List<LoggedDashboardChooser<AutoTask>> choosers = new ArrayList<>();

    /** Maximum number of sequential task selections supported. */
    static final int NUM_CHOOSERS = 12;

    /**
     * Constructs the AutoSelector, wires the trajectory factory, pre-loads all
     * Choreo trajectories, and initializes the start-position chooser.
     *
     * @param drive drive subsystem (used for pose getter/setter and field visualization)
     */
    public AutoSelector(Drive drive) {
        factory = new AutoFactory(
            drive::getPose,
            drive::setPose,
            s -> {},
            true,
            drive,
            (traj, starting) -> {
                // Called whenever a Choreo trajectory starts or finishes.
                // On finish: clear the trajectory visualization.
                // On start: log and visualize the full trajectory path.
                if (!starting) {
                    Pose2d[] empty = new Pose2d[0];
                    Logger.recordOutput("Auto/Trajectory", empty);
                    drive.field.getObject("Auto/Trajectory").setPoses(empty);
                    return;
                }

                Pose2d[] trj = traj.getPoses();
                Pose2d[] flipped = AllianceUtil.autoflip(trj).orElse(trj);
                Logger.recordOutput("Auto/Trajectory", flipped);
                drive.field.getObject("Auto/Trajectory").setPoses(flipped);
            }
        );

        // Pre-load all trajectories into the factory cache to avoid I/O at match start.
        for (ChoreoTraj traj : ChoreoTraj.ALL_TRAJECTORIES.values()) {
            factory.cache().loadTrajectory(traj.name());
        }

        // Create all choosers upfront; only the first is populated here.
        for (int i = 0; i < NUM_CHOOSERS; i++) {
            LoggedDashboardChooser<AutoTask> c = chooser(i);
            choosers.add(c);
        }

        // Populate chooser 0 with the three starting positions.
        LoggedDashboardChooser<AutoTask> s0 = choosers.get(0);
        add(s0, AutoTask.StartPos1);
        add(s0, AutoTask.StartPos2);
        add(s0, AutoTask.StartPos3);
        add(s0, AutoTask.Cancel);

        s0.onChange(node -> update(0, node));
    }

    /**
     * Adds {@code n} to chooser {@code c} using its {@link AutoTask#label()} as the
     * display string.
     */
    private void add(LoggedDashboardChooser<AutoTask> c, AutoTask n) {
        c.addOption(n.label(), n);
    }

    /**
     * Handles a selection change on chooser {@code n}.
     *
     * <ol>
     *   <li>If a later selection already exists at index {@code n}, {@link #cutoff(int)}
     *       tears down all choosers and entries from index {@code n+1} onward, then
     *       removes the stale entry at index {@code n} from {@link #current}.
     *   <li>Appends {@code change} to {@link #current} at index {@code n}.
     *   <li>Populates chooser {@code n+1} with all {@link AutoTask} values plus
     *       {@link AutoTask#Cancel}, and registers this method recursively as its listener.
     * </ol>
     *
     * <p>If {@code n} is the last chooser index ({@value #NUM_CHOOSERS}{@code  - 1}),
     * the method returns immediately — there is no next chooser to update.
     *
     * @param n      zero-based index of the chooser that changed
     * @param change the newly selected step
     */
    public void update(int n, AutoTask change) {
        if (n >= NUM_CHOOSERS - 1) return;

        if (current.size() > n) {
            cutoff(n + 1);
            current.remove(n);
        }

        current.add(change);

        Set<AutoTask> next = EnumSet.allOf(AutoTask.class);
        LoggedDashboardChooser<AutoTask> s = choosers.get(n + 1);

        for (AutoTask node : next) add(s, node);
        add(s, AutoTask.Cancel);
        s.onChange(node -> update(n + 1, node));
    }

    /**
     * Resets all choosers and current-selection entries from index {@code n} onward.
     *
     * <p>Each chooser's underlying {@link edu.wpi.first.wpilibj.smartdashboard.SendableChooser}
     * is closed and replaced with a fresh instance so the dashboard reflects the reset.
     * The corresponding entries in {@link #current} are bulk-removed.
     *
     * <p>Called by {@link #update} whenever the driver changes a selection that has
     * already cascaded to later choosers.
     *
     * @param n index of the first chooser to reset (inclusive)
     */
    private void cutoff(int n) {
        current.subList(n, current.size()).clear();

        while (n < choosers.size()) {
            choosers.get(n).getSendableChooser().close();
            LoggedDashboardChooser<AutoTask> next = chooser(n);
            choosers.set(n, next);
            n++;
        }
    }

    /**
     * Creates a fresh, empty {@link LoggedDashboardChooser} for position {@code n}.
     *
     * <p>Chooser 0 is named {@code Auto/StartPos}; all others are named
     * {@code Auto/Task}<i>n</i>.
     *
     * @param n zero-based chooser index
     * @return a new, unpopulated chooser
     */
    private LoggedDashboardChooser<AutoTask> chooser(int n) {
        if (n == 0) return new LoggedDashboardChooser<>("Auto/StartPos");
        else return new LoggedDashboardChooser<>("Auto/Task" + n);
    }

    /**
     * Assembles a {@link Command} that executes all currently selected steps in order.
     *
     * <p>For each step in {@link #current} (up to but not including the first
     * {@link AutoTask#Cancel}):
     * <ol>
     *   <li>If the step has a non-null {@link AutoTask#resetTo}, prepends a
     *       {@code Commands.runOnce} that calls {@link Drive#setPose} with the
     *       alliance-flipped pose, ensuring odometry is correct before the step runs.
     *   <li>Appends the step's command via {@link AutoTask#full}.
     * </ol>
     *
     * <p>Returns {@link Commands#none()} if no selections have been made.
     *
     * @param drive     drive subsystem
     * @param shooter   shooter subsystem
     * @param spindexer spindexer subsystem
     * @return the assembled autonomous command sequence
     */
    public Command build(Drive drive, Shooter shooter, Spindexer spindexer) {
        Command command = Commands.none();

        int end = current.size() - 1;
        if (end < 0) return command;
        if (current.get(end) != AutoTask.Cancel) current.add(AutoTask.Cancel);

        System.out.println(current);

        for (int i = 0; i < current.size(); i++) {
            AutoTask node = current.get(i);

            if (node == AutoTask.Cancel) break;

            if (node.resetTo != null) {
                Pose2d fieldPose = AllianceUtil.unsafe.autoflip(node.resetTo);

                drive.setPose(fieldPose);
                command = Commands.sequence(
                    command,
                    Commands.runOnce(() -> drive.setPose(fieldPose), drive)
                );
            }

            command = Commands.sequence(
                command,
                node.full(drive, shooter, spindexer)
            );
        }

        return command;
    }
}
