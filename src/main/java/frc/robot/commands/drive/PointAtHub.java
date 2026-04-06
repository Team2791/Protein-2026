package frc.robot.commands.drive;

import static frc.robot.util.MathPlus.kTau;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.GameConstants;
import frc.robot.constants.RuntimeConstants;
import frc.robot.constants.RuntimeConstants.Mode;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Vec2;
import org.littletonrobotics.junction.Logger;

/**
 * Command that rotates the robot to face the hub while allowing manual
 * translational control via joystick.
 *
 * <p>
 * Uses a PID controller on heading error with continuous input wrapping
 * to smoothly track the hub's direction from the robot's current pose.
 * Alliance-aware: computes the target angle in the blue coordinate frame
 * and autoflips the robot pose as needed.
 */
public class PointAtHub extends Command {

    final Drive drive;
    final Shooter shooter;
    final JoystickDrive jsd;

    /** PID controller for rotation toward hub. */
    final PIDController ctl = new PIDController(
        ControlConstants.Nearby.kTurnP,
        ControlConstants.Nearby.kTurnI,
        ControlConstants.Nearby.kTurnD
    );

    public PointAtHub(Drive drive, Shooter shooter, CommandXboxController ctl) {
        this.drive = drive;
        this.shooter = shooter;
        this.jsd = ctl == null ? null : new JoystickDrive(ctl, drive);
        addRequirements(drive); // shooter read-only in this command

        this.ctl.enableContinuousInput(0, kTau);
        this.ctl.setTolerance(ControlConstants.Nearby.kRotationTolerance);

        SmartDashboard.putData("Point/RotPID", this.ctl);
    }

    public PointAtHub(Drive drive, Shooter shooter) {
        this(drive, shooter, null);
    }

    /**
     * @param r robot center
     * @param g goal pose
     * @param theta robot heading
     * @return
     */
    double wantedAngle(Vec2 r, Vec2 g, Rotation2d theta) {
        Vec2 d = new Vec2(ShooterConstants.kBotToShooter.getTranslation());
        double h = theta.getRadians();

        // newtonian iteration
        for (int i = 0; i < 10; i++) {
            h = g.sub(r).sub(d.rotate(h)).atan2();
        }

        return h;
    }

    @Override
    public void execute() {
        // relevant positions
        Vec2 botBlue = new Vec2(AllianceUtil.unsafe.autoflip(drive.getPose()));
        Rotation2d botTheta = AllianceUtil.unsafe.autoflip(drive.getRotation());
        Vec2 hub = new Vec2(GameConstants.Objects.kHub);

        // bot velocity
        Vec2 vel = new Vec2(drive.getFieldSpeeds());
        Vec2 velBlue = AllianceUtil.unsafe.invert() ? vel.neg() : vel;

        // shooter velocity
        double shootFactor = -shooter.data().leader.velocity() / 50;
        if (RuntimeConstants.kCurrentMode == Mode.SIM) shootFactor = 5.7;

        // distance to hub
        double bot2hub = botBlue.sub(hub).mag();

        // offset factor
        double factor = (ShooterConstants.kAimFactor * bot2hub) / shootFactor;
        Vec2 offset = velBlue.mul(factor);
        Vec2 aimAt = hub.sub(offset);

        double theta = wantedAngle(botBlue, aimAt, botTheta);
        double rot = ctl.calculate(botTheta.getRadians(), theta);

        Vec2 linear = jsd == null ? new Vec2(0, 0) : jsd.linear();

        // logging
        Logger.recordOutput(
            "Drive/PointAtHub/Target",
            botBlue.wpi(new Rotation2d(theta))
        );
        Logger.recordOutput("Drive/PointAtHub/Measured", botBlue.wpi(botTheta));
        Logger.recordOutput(
            "Drive/PointAtHub/HubTarget",
            aimAt.wpi(new Rotation2d())
        );

        drive.drive(new ChassisSpeeds(linear.x, linear.y, rot));
    }
}
