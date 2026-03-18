package frc.robot.commands.drive;

import static frc.robot.util.MathPlus.kTau;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.GameConstants;
import frc.robot.subsystems.drive.Drive;
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
    final JoystickDrive jsd;

    /** PID controller for rotation toward hub. */
    final PIDController ctl = new PIDController(
        ControlConstants.Nearby.kTurnP,
        ControlConstants.Nearby.kTurnI,
        ControlConstants.Nearby.kTurnD
    );

    public PointAtHub(Drive drive, CommandXboxController ctl) {
        this.drive = drive;
        this.jsd = ctl == null ? null : new JoystickDrive(ctl, drive);
        addRequirements(drive);

        this.ctl.enableContinuousInput(0, kTau);
        this.ctl.setTolerance(
            ControlConstants.Nearby.kTolerance.getRotation().getRadians()
        );

        SmartDashboard.putData("Point/RotPID", this.ctl);
    }

    public PointAtHub(Drive drive) {
        this(drive, null);
    }

    @Override
    public void execute() {
        Pose2d pose = AllianceUtil.unsafe.autoflip(drive.getPose());
        Pose2d hub = GameConstants.Objects.kHub;
        Vec2 delta = new Vec2(pose).sub(new Vec2(hub));
        Rotation2d theta = delta.theta().plus(Rotation2d.kPi);
        double rot = ctl.calculate(
            pose.getRotation().getRadians(),
            theta.getRadians()
        );
        Vec2 linear = jsd == null ? new Vec2(0, 0) : jsd.linear();

        Logger.recordOutput(
            "Drive/PointAtHub/Target",
            new Pose2d(pose.getTranslation(), theta)
        );
        Logger.recordOutput("Drive/PointAtHub/Measured", pose);

        drive.drive(new ChassisSpeeds(linear.x, linear.y, rot));
    }
}
