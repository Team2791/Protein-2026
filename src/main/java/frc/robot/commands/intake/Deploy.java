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
public class Deploy extends Command {

    final Intake intake;
    final boolean deployed;

    /**
     * Creates a Deploy command.
     *
     * @param intake   The intake subsystem
     * @param deployed {@code true} to deploy, {@code false} to retract
     */
    public Deploy(Intake intake, boolean deployed) {
        this.intake = intake;
        this.deployed = deployed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.deploy(deployed);
    }

    @Override
    public boolean isFinished() {
        return intake.pivotData().leader.pid().atTarget();
    }
}
