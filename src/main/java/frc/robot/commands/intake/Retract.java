package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/**
 * Command that deploys or retracts the intake and waits until the pivot
 * reaches its target position.
 *
 * <p>
 * Finishes once the pivot PID reports at-target (deployed or retracted).
 */
public class Retract extends Command {

    final Intake intake;

    /**
     * Creates a Deploy command.
     *
     * @param intake   The intake subsystem
     * @param deployed {@code true} to deploy, {@code false} to retract
     */
    public Retract(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.deploy(false);
    }

    @Override
    public void end(boolean interrupted) {
        intake.deploy(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
