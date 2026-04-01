package frc.robot.commands.drive.pathfind;

import static frc.robot.util.MathPlus.kTau;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drive.Drive;

public class Point extends Command {

    static class TunableController extends PIDController {

        static final TunableController instance = new TunableController();

        private TunableController() {
            super(
                ControlConstants.Nearby.kTurnP,
                ControlConstants.Nearby.kTurnI,
                ControlConstants.Nearby.kTurnD
            );
            super.enableContinuousInput(0, kTau);
            super.setTolerance(ControlConstants.Nearby.kRotationTolerance);

            SmartDashboard.putData("PID/Point", this);
        }
    }

    final Rotation2d target;

    final PIDController ctl = TunableController.instance;

    /** Drive subsystem for motion control. */
    final Drive drive;

    public Point(Drive drive, Rotation2d target) {
        this.drive = drive;
        this.target = target;

        addRequirements(drive);
    }

    @Override
    public final void execute() {
        Pose2d robot = drive.getPose();
        double power = ctl.calculate(
            robot.getRotation().getRadians(),
            target.getRadians()
        );
        // Send robot-relative speeds to drive
        drive.drive(new ChassisSpeeds(0, 0, power));
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return (
            ctl.atSetpoint() &&
            drive.getChassisSpeeds().omegaRadiansPerSecond < 0.3
        );
    }
}
