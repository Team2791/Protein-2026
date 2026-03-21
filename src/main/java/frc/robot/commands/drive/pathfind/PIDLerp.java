package frc.robot.commands.drive.pathfind;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
            xctl.setTolerance(ControlConstants.Nearby.kTolerance.getX());
            yctl.setTolerance(ControlConstants.Nearby.kTolerance.getY());

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
            boolean ok = AStar.query(us, tt);
            if (!ok) break;
        }

        farthest = Math.min(farthest, path.length - 1);
    }

    @Override
    public void initialize() {
        Vec2 bot = blue();
        this.path = AStar.path(bot, new Vec2(target));

        Logger.recordOutput(
            "PIDLerp/Direct",
            new Translation2d[] { bot.wpi(), target.getTranslation() }
        );

        Logger.recordOutput(
            "PIDLerp/Path",
            Arrays.stream(this.path)
                .map(v -> v.wpi())
                .toArray(Translation2d[]::new)
        );
    }

    @Override
    public void execute() {
        search();

        Vec2 bot = blue();
        Vec2 target = path[farthest];
        Vec2 speeds = ctl.commanded(bot, target);

        if (speeds.mag() > maxVel) {
            speeds = speeds.norm().mul(maxVel);
        }

        Logger.recordOutput(
            "PIDLerp/CurrentLerp",
            new Translation2d[] { bot.wpi(), target.wpi() }
        );

        drive.drive(new ChassisSpeeds(speeds.x, speeds.y, 0));
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return ctl.atTarget() && farthest == path.length - 1;
    }
}
