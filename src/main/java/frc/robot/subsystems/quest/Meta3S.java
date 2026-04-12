package frc.robot.subsystems.quest;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;
import gg.questnav.questnav.QuestNav;

/**
 * Meta 3S implementation of {@link QuestIO} for Meta Quest-based pose estimation.
 *
 * <p>
 * This class interfaces with the Meta 3S device using the QuestNav library
 * to obtain pose frames for vision-based localization.
 */
public class Meta3S extends QuestIO {

    /** The QuestNav instance for pose tracking. */
    final QuestNav quest;

    static String runcmd(String... command) {
        try {
            return new String(
                new ProcessBuilder(command)
                    .redirectErrorStream(true)
                    .start()
                    .getInputStream()
                    .readAllBytes()
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    static void tryForceReconnect() {
        String lsdevice = runcmd("/usr/bin/adb", "devices");
        boolean connected = lsdevice.contains("5555");

        if (!connected) {
            runcmd("/usr/bin/adb", "connect", "10.27.91.200:5555");
        }

        runcmd(
            "/usr/bin/adb",
            "shell",
            "am",
            "start",
            "-n",
            "gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity"
        );
    }

    /**
     * Constructs a Meta3S vision system.
     *
     * <p>
     * Initializes the QuestNav instance. {@link QuestIO#data} will be updated
     * with the latest readings on each {@link update()} call.
     */
    public Meta3S() {
        this.quest = new QuestNav();

        quest.onConnected(() -> {
            Elastic.sendNotification(
                new Elastic.Notification(
                    NotificationLevel.INFO,
                    "Quest - Connected",
                    "Quest has connected"
                )
            );

            new Thread(() -> {
                tryForceReconnect();
            })
                .start();
        });

        quest.onTrackingAcquired(() -> {
            Elastic.sendNotification(
                new Elastic.Notification(
                    NotificationLevel.INFO,
                    "Quest - Tracking",
                    "Quest has tracking"
                )
            );
        });

        quest.onDisconnected(() -> {
            Elastic.sendNotification(
                new Elastic.Notification(
                    NotificationLevel.ERROR,
                    "Quest - Disconnected",
                    "Quest has disconnected"
                )
            );

            new Thread(() -> {
                tryForceReconnect();
            })
                .start();
        });

        quest.onTrackingLost(() -> {
            Elastic.sendNotification(
                new Elastic.Notification(
                    NotificationLevel.ERROR,
                    "Quest - Tracking Lost",
                    "Quest has lost tracking"
                )
            );
        });

        quest.onLowBattery(25, battery -> {
            Elastic.sendNotification(
                new Elastic.Notification(
                    NotificationLevel.WARNING,
                    "Quest - Low Battery",
                    String.format("Quest has low battery (%d%%)", battery)
                )
            );
        });
    }

    @Override
    public void update() {
        quest.commandPeriodic();

        data.connected = quest.isConnected();
        data.tracking = quest.isTracking() && quest.isConnected();
        data.latency = Milliseconds.of(quest.getLatency());
        data.readings = quest.getAllUnreadPoseFrames();
        data.battery = quest.getBatteryPercent().orElse(-1);
    }

    @Override
    protected void reset(Pose3d pose) {
        quest.setPose(pose);
    }
}
