package frc.robot.subsystems.quest;

import static frc.robot.constants.VisionConstants.kBotToQuest;
import static frc.robot.constants.VisionConstants.kQuestDevs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.GameConstants;
import frc.robot.subsystems.quest.QuestIO.QuestData;
import frc.robot.util.Vec2;
import frc.robot.util.VisionMeasurement;
import gg.questnav.questnav.PoseFrame;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/**
 * QuestNav vision subsystem for Meta Quest-based pose estimation.
 *
 * <p>
 * This subsystem provides additional vision-based localization using a Meta Quest device
 * running QuestNav. Like Photon, this does not extend SubsystemBase because it operates
 * independently of the command scheduler.
 *
 * <p>
 * The Quest system:
 * <ul>
 *   <li>Uses inside-out pose tracking from the Quest device
 *   <li>Reports pose measurements to the drivetrain's odometry system
 *   <li>Can improve localization accuracy in GPS-denied environments
 *   <li>Provides independent verification of robot position
 * </ul>
 */
public class Quest {

    /** The QuestNav instance for pose tracking. */
    final QuestIO quest;

    /** Callback to send vision measurements to the drivetrain. */
    final Consumer<VisionMeasurement> addMeasurement;

    /**
     * Constructs a Quest vision system.
     *
     * Initializes the QuestNav instance and registers periodic updates.
     * Vision measurements will be sent to the provided callback function.
     *
     * @param quest The QuestNav IO implementation
     * @param addMeasurement Callback for reporting vision measurements
     */
    public Quest(QuestIO quest, Consumer<VisionMeasurement> addMeasurement) {
        this.quest = quest;
        this.addMeasurement = addMeasurement;
        AutoLogOutputManager.addObject(this);
    }

    @AutoLogOutput(key = "Quest/Ok")
    public boolean ok() {
        return this.quest.data.connected;
    }

    /**
     * Resets the Quest NavBoard's pose estimate.
     *
     * Converts the robot pose to the Quest frame of reference and sets it.
     * Should be called when the robot's true position is known (e.g., at start of match).
     *
     * @param pose The robot's current pose to sync with Quest
     */
    public void reset(Pose2d pose) {
        // Convert 2D pose to 3D
        Pose3d bot = new Pose3d(pose);

        // Transform to Quest frame of reference
        Pose3d quest = bot.transformBy(kBotToQuest);

        // Update Quest's pose estimate
        this.quest.reset(quest);
    }

    /**
     * Get the Quest's data
     * @return a copy of the quest's data
     */
    public QuestData data() {
        return quest.data.clone();
    }

    /**
     * Get the latest heading from the Quest device.
     * @return The latest heading, if available
     */
    public Optional<Rotation2d> heading() {
        return botPose().map(pose -> pose.getRotation());
    }

    public Optional<Pose2d> botPose() {
        PoseFrame[] frames = this.quest.data.readings;
        PoseFrame[] tracking = Arrays.stream(frames)
            .filter(PoseFrame::isTracking)
            .toArray(PoseFrame[]::new);

        if (tracking.length == 0) return Optional.empty();

        return Optional.of(
            tracking[tracking.length - 1].questPose3d()
                .transformBy(kBotToQuest.inverse())
                .toPose2d()
        );
    }

    boolean reset = false;

    /**
     * Periodic update method called every robot tick.
     *
     * Retrieves new pose frames from the Quest device and converts them to
     * vision measurements for the drivetrain's odometry system.
     */
    public void update() {
        // Refresh Quest data
        this.quest.update();

        if (!reset && quest.data.connected && quest.data.tracking) {
            reset = true;
            reset();
        }

        // Record all outputs
        Logger.processInputs("Quest", this.quest.data);

        // Get all new pose frames from Quest since last update
        PoseFrame[] frames = this.quest.data.readings;
        if (frames.length == 0) return;

        // Process each pose frame
        for (PoseFrame frame : frames) {
            // Skip frames that don't have valid tracking
            if (!frame.isTracking()) continue;

            // Extract pose data from frame
            double ts = frame.dataTimestamp();
            Pose3d quest = frame.questPose3d();

            // Transform from Quest frame to robot frame
            Pose3d bot = quest.transformBy(kBotToQuest.inverse());

            // Create and report vision measurement
            VisionMeasurement est = new VisionMeasurement(bot, kQuestDevs, ts);
            addMeasurement.accept(est);
        }
    }

    private void reset() {
        // check if quest is within perimeter
        if (quest.data.readings.length == 0) return;

        PoseFrame f = quest.data.readings[0];
        Pose3d bot = f.questPose3d().transformBy(kBotToQuest.inverse());
        Vec2 bot2 = new Vec2(bot.toPose2d());

        // check persistence
        if (bot2.ltstrict(new Vec2(GameConstants.kRedOrigin))) return;

        quest.reset(new Pose3d());
    }
}
