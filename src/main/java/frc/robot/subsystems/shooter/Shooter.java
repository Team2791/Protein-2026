package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GameConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterIO.ShooterData;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Vec2;

/**
 * Shooter subsystem for controlling the robot's ball-shooting mechanism.
 *
 * <p>This subsystem wraps a {@link ShooterIO} implementation to provide a consistent interface
 * for controlling the shooter regardless of whether running on real hardware, in simulation,
 * or in log replay.
 *
 * <p>Automatic speed control is handled by the IO layer based on robot position.
 * Manual override is available via {@link #setVelocity(double)} for debugging.
 */
public class Shooter extends SubsystemBase {

    /** The hardware/simulation IO interface for the shooter. */
    final ShooterIO io;

    /** The drive subsystem */
    final Drive drive;

    /**
     * When {@code true}, the shooter is in manual mode (velocity set via {@link #setVelocity}).
     * In manual mode, the automatic distance-based control in {@link #update()} is bypassed.
     */
    boolean manual = false;

    /**
     * Constructs a Shooter subsystem with the given IO implementation.
     *
     * @param io The IO implementation to use (real hardware, replay, etc.)
     */
    public Shooter(ShooterIO io, Drive drive) {
        this.io = io;
        this.drive = drive;
    }

    /**
     * Returns a snapshot of the current shooter sensor data.
     *
     * @return A clone of the current {@link ShooterData}
     */
    public ShooterData data() {
        return io.data.clone();
    }

    /**
     * DEBUGGING: Sets the shooter wheel velocity directly, bypassing automatic control.
     *
     * @param velocity The target velocity in radians/second
     */
    public void setVelocity(double velocity) {
        this.manual = true;
        io.setVelocity(velocity);
    }

    /** Calls {@link ShooterIO#update()} every robot loop to refresh sensor data. */
    @Override
    public void periodic() {
        io.update();

        if (manual) return;

        // Automatic control logic
        Pose2d pose = drive.getPose();
        Pose2d blue = AllianceUtil.autoflip(pose).orElse(pose);

        if (blue.getX() > ShooterConstants.kSpinUpThreshold) {
            io.setVelocity(0);
            return;
        }

        Vec2 hub = new Vec2(GameConstants.Objects.kHub);
        Vec2 delta = hub.sub(new Vec2(blue));
        double dist = delta.mag();

        // regress
        double vel = ShooterConstants.Regression.apply(dist);
        io.setVelocity(vel);
    }
}
