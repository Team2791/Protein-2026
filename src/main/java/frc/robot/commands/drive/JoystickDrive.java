package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil;
import frc.robot.util.RateLimiter;
import frc.robot.util.Vec2;
import java.util.Optional;

/**
 * Command for driving the robot using joystick input from an Xbox controller.
 *
 * <p>This command
 * <ol>
 *  <li>Reads joystick inputs for translation and rotation
 *  <li>Applies deadbands and input shaping for finer control
 *  <li>Implements rate limiting for smooth acceleration
 *  <li>Transforms inputs to field-centric coordinates based on alliance color
 *  <li>Scales inputs to maximum speed limits
 *  <li>Commands the drive subsystem with the calculated chassis speeds
 * </ol>
 */
public class JoystickDrive extends Command {

    /** The Xbox controller used for driving */
    final CommandXboxController ctl;

    /** The drive subsystem to control */
    final Drive drive;

    /** Rate limiter for smoothing joystick inputs */
    final RateLimiter slew;

    public JoystickDrive(CommandXboxController ctl, Drive drive) {
        this.ctl = ctl;
        this.drive = drive;
        this.slew = new RateLimiter(
            ControlConstants.SlewRateLimit.kOrthogonal,
            ControlConstants.SlewRateLimit.kOrthogonal,
            ControlConstants.SlewRateLimit.kRotational
        );

        addRequirements(drive);
    }

    @Override
    public void execute() {
        Vec2 linear = new Vec2(ctl.getLeftX(), ctl.getLeftY());

        // apply deadband to joystick inputs
        double linearMag = MathUtil.applyDeadband(
            linear.mag(),
            IOConstants.Controller.kDeadband
        );
        double rot = MathUtil.applyDeadband(
            ctl.getRightX(),
            IOConstants.Controller.kDeadband
        );

        Vec2 input = linear.norm().mul(linearMag);

        // apply squared inputs for finer control at low speeds
        Vec2 input2 = input.mul(input.abs());
        double rot2 = rot * Math.abs(rot);

        // apply rate limiting for smooth acceleration
        RateLimiter.Outputs limited = slew.calculate(input2, rot2);

        /*
         * Controller to WPI coordinate system conversion:
         *
         * Controller coordinates:    WPI coordinates:
         *   +X = right                 +X = forward
         *   +Y = down                  +Y = left
         *   +Rot = clockwise           +Rot = counter-clockwise
         *
         * Transformation needed:
         *   +Yc -> -Xw  (down stick -> forward motion negation)
         *   +Xc -> -Yw  (right stick -> left motion negation)
         *   +Rotc -> -Rotw (negate rotation for ccw-positive)
         */
        Vec2 linearcmd = new Vec2(-limited.vel().y, -limited.vel().x);
        double rotcmd = -limited.rot();

        /*
         * Field-centric drive coordinate transformation based on alliance color.
         *
         * BLUE ALLIANCE (origin on driver station side):
         *   - Joystick forward (+Y) -> Robot moves toward opposing alliance (+X field coords)
         *   - No transformation needed - joystick and field coordinates align naturally
         *
         * RED ALLIANCE (origin on opposing alliance side):
         *   - Joystick forward (+Y) -> Robot should move toward opposing alliance (-X field coords)
         *   - Linear commands must be inverted to maintain driver perspective
         *   - This ensures drivers have consistent controls regardless of alliance color
         */
        Optional<Boolean> invert = AllianceUtil.invert();
        if (invert.orElse(false)) linearcmd = linearcmd.neg(); // invert for Red alliance, skip if no FMS/DS

        // scale to max speeds
        Vec2 vel = linearcmd.mul(ControlConstants.Drive.kMaxLinearSpeed);
        double omega = rotcmd * ControlConstants.Drive.kMaxAngularSpeed;

        // no need to speed limit for diagonals, drive.runVelocity() handles that.
        // just send the desired chassis speeds
        drive.drive(new ChassisSpeeds(vel.x, vel.y, omega));
    }
}
