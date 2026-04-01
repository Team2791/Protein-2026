package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants.Roller.RollerState;
import frc.robot.subsystems.intake.Intake;

public class Roll extends Command {

    final Intake intake;
    final RollerState roll;

    public Roll(Intake intake, RollerState running) {
        this.intake = intake;
        this.roll = running;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.roll(roll);
    }

    @Override
    public void end(boolean interrupted) {
        intake.roll(RollerState.kNormal);
    }
}
