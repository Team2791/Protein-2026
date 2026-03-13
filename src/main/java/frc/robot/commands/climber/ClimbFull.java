package frc.robot.commands.climber;

import static frc.robot.constants.ClimberConstants.Position.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;

/** Full teleop climb cycle */
public class ClimbFull extends SequentialCommandGroup {

    public ClimbFull(Climber climber, Intake intake) {
        addCommands(
            new Engage(climber, intake, true, true),
            new SetPosition(climber, EXPAND),
            new SetPosition(climber, CONTRACT), // <- L1 achieved
            new SetPosition(climber, EXPAND),
            new SetPosition(climber, CONTRACT), // <- L2 achieved
            new SetPosition(climber, EXPAND),
            new SetPosition(climber, CONTRACT) // <- L3 achieved
        );
    }
}
