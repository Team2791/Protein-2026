package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RunIntake extends Command {

    final Intake intake;

    public RunIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public void initialize() {
        intake.run(true);
    }

    public void end() {
        intake.run(false);
    }
}
