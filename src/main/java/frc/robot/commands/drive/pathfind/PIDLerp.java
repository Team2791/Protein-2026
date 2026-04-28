package frc.robot.commands.drive.pathfind;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AStar;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Vec2;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

/**
 * Use in conjunction with {@link Point} for a complete pathfind
 */
public class PIDLerp extends Command {

    static class TunablePID {

        final PIDController xctl = new PIDController(
            ControlConstants.Nearby.kOrthoP,
            ControlConstants.Nearby.kOrthoI,
            ControlConstants.Nearby.kOrthoD
        );

        final PIDController yctl = new PIDController(
            ControlConstants.Nearby.kOrthoP,
            ControlConstants.Nearby.kOrthoI,
            ControlConstants.Nearby.kOrthoD
        );

        public TunablePID() {
            SmartDashboard.putData("PID/PIDLerpX", xctl);
            SmartDashboard.putData("PID/PIDLerpY", yctl);
        }

        public Vec2 commanded(Vec2 current, Vec2 target) {
            double x = xctl.calculate(current.x, target.x);
            double y = yctl.calculate(current.y, target.y);
            Vec2 ret = new Vec2(x, y);

            if (AllianceUtil.unsafe.invert()) return ret.neg();
            else return ret;
        }

        public boolean atTarget() {
            return xctl.atSetpoint() && yctl.atSetpoint();
        }
    }

    final Pose2d target;
    final Drive drive;
    final double maxVel;
    final TunablePID ctl = new TunablePID();

    Vec2[] path = new Vec2[0];
    int farthest = 0;

    /**
     * Target is blue-side pose thanks.
     *
     * @see frc.robot.auto.AutoNode
     */
    public PIDLerp(Drive drive, Pose2d target) {
        this(drive, target, DriveConstants.maxSpeedMetersPerSec);
    }

    /**
     * Target is blue-side pose thanks.
     *
     * @see frc.robot.auto.AutoNode
     */
    public PIDLerp(Drive drive, Pose2d target, double maxVel) {
        this.drive = drive;
        this.target = target;
        this.maxVel = maxVel;

        addRequirements(drive);
    }

    Vec2 blue() {
        return new Vec2(AllianceUtil.unsafe.autoflip(drive.getPose()));
    }

    void search() {
        Vec2 us = blue();

        for (; farthest < path.length; farthest++) {
            Vec2 tt = path[farthest];
            boolean ok = AStar.raycast(us, tt);
            if (!ok) break;
        }

        farthest = Math.min(farthest, path.length - 1);
    }

    boolean atlast() {
        return farthest == path.length - 1;
    }

    @Override
    public void initialize() {
        Vec2 bot = blue();
        this.path = AStar.path(bot, new Vec2(target));

        Logger.recordOutput(
            "PIDLerp/Direct",
            new Translation2d[] { bot.wpi(), target.getTranslation() }
        );

        drive.field
            .getObject("PIDLerp/Direct")
            .setPoses(new Pose2d[] { bot.wpi(new Rotation2d()), target });

        Logger.recordOutput(
            "PIDLerp/Path",
            Arrays.stream(this.path)
                .map(v -> v.wpi())
                .toArray(Translation2d[]::new)
        );

        drive.field
            .getObject("PIDLerp/Path")
            .setPoses(
                Arrays.stream(this.path)
                    .map(v -> v.wpi(new Rotation2d()))
                    .toList()
            );
    }

    @Override
    public void execute() {
        search();

        Vec2 bot = blue();
        Vec2 target = atlast() ? new Vec2(this.target) : path[farthest];
        Vec2 speeds = ctl.commanded(bot, target);

        if (speeds.mag() > maxVel) {
            speeds = speeds.norm().mul(maxVel);
        } else if(speeds.mag() <0.75 && !atlast()) { // set minimum speed
            speeds = speeds.norm().mul(0.75);
        }

        Logger.recordOutput(
            "PIDLerp/CurrentLerp",
            new Translation2d[] { bot.wpi(), target.wpi() }
        );

        drive.field
            .getObject("PIDLerp/CurrentLerp")
            .setPoses(
                new Pose2d[] {
                    bot.wpi(new Rotation2d()),
                    target.wpi(new Rotation2d()),
                }
            );

        drive.drive(new ChassisSpeeds(speeds.x, speeds.y, 0));
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds());

        drive.field.getObject("PIDLerp/Direct").setPoses();
        drive.field.getObject("PIDLerp/CurrentLerp").setPoses();
        drive.field.getObject("PIDLerp/Path").setPoses();
    }

    @Override
    public boolean isFinished() {
        double posErr = blue().sub(new Vec2(target)).mag();

        return (
            posErr < ControlConstants.PIDLerp.kRotationTolerance &&
            atlast() &&
            new Vec2(drive.getChassisSpeeds()).mag() < 0.1
        );
    }
}
