package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

public class SetShooter extends Command {

    final Shooter shooter;
    final ShooterConstants.Setpoint sp;

    public SetShooter(Shooter shooter, ShooterConstants.Setpoint sp) {
        this.shooter = shooter;
        this.sp = sp;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.manual(sp);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
