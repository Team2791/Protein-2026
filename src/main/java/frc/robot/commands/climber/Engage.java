package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.Deploy;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;

/**
 * Command that engages or disengages the climber pneumatic hooks.
 *
 * <p>
 * Sequence:
 * <ol>
 *   <li>If engaging anything: retract the intake and wait for retraction
 *   <li>Actuate solenoids and wait {@link ClimberConstants#kEngageDuration}
 *   <li>If disengaging everything: deploy the intake
 * </ol>
 */
class Engage extends SequentialCommandGroup {

    /**
     * Creates an Engage command.
     *
     * @param climber The climber subsystem
     * @param intake  The intake subsystem
     * @param inner   {@code true} to engage the inner hook, {@code false} to disengage
     * @param outer   {@code true} to engage both outer hooks, {@code false} to disengage
     */
    public Engage(
        Climber climber,
        Intake intake,
        boolean inner,
        boolean outer
    ) {
        boolean engaging = inner || outer;

        if (engaging) {
            addCommands(new Deploy(intake, false));
        }

        addCommands(
            new Command() {
                final Timer timer = new Timer();

                {
                    addRequirements(climber);
                }

                @Override
                public void initialize() {
                    climber.setInner(inner);
                    climber.setOuter(outer);
                    timer.restart();
                }

                @Override
                public void end(boolean interrupted) {
                    timer.stop();
                }

                @Override
                public boolean isFinished() {
                    return timer.get() >= ClimberConstants.kEngageDuration;
                }
            }
        );

        if (!engaging) {
            addCommands(new Deploy(intake, true));
        }
    }
}
