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
    final PIDController rotController = new PIDController(
        ControlConstants.Nearby.kTurnP,
        ControlConstants.Nearby.kTurnI,
        ControlConstants.Nearby.kTurnD
    );

    public PointAtHub(Drive drive, CommandXboxController ctl) {
        this.drive = drive;
        this.jsd = new JoystickDrive(ctl, drive);
        addRequirements(drive);

        rotController.enableContinuousInput(0, kTau);
        rotController.setTolerance(
            ControlConstants.Nearby.kTolerance.getRotation().getRadians()
        );

        SmartDashboard.putData("Point/RotPID", rotController);
    }

    @Override
    public void execute() {
        Pose2d posBlue = AllianceUtil.unsafe.autoflip(drive.getPose());
        Pose2d blueHub = GameConstants.Objects.kHub;
        Vec2 delta = new Vec2(posBlue).sub(new Vec2(blueHub));
        Rotation2d theta = delta.theta().plus(Rotation2d.kPi);
        double rot = rotController.calculate(
            posBlue.getRotation().getRadians(),
            theta.getRadians()
        );
        Vec2 linear = jsd.linear();

        Logger.recordOutput(
            "Point/Rot",
            new Pose2d(posBlue.getTranslation(), theta)
        );
        Logger.recordOutput("Point/Cur", posBlue);

        drive.drive(new ChassisSpeeds(linear.x, linear.y, rot));
    }
}
