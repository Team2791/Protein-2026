package frc.robot.commands.climber;

import static frc.robot.constants.ClimberConstants.Position.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;

/**
 * Autonomous L1 climb sequence.
 *
 * <p>
 * Engages the inner hooks, expands the axle, then partially contracts so
 * the robot can descend later. Sequence:
 *
 * <ol>
 *   <li>Engage inner hooks (outer disengaged)
 *   <li>Expand axle
 *   <li>Partially contract axle
 * </ol>
 */
public class ClimbAuto extends SequentialCommandGroup {

    public ClimbAuto(Climber climber, Intake intake) {
        addCommands(
            new Engage(climber, intake, true, false),
            new SetPosition(climber, EXPAND),
            new SetPosition(climber, CONTRACT_PARTIAL)
        );
    }
}
