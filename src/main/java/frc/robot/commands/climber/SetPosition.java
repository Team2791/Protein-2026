package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;

/**
 * Command that sets the climber axle to a target position and holds it.
 *
 * <p>
 * This command runs until interrupted (e.g. by a button release or a
 * sequential composition), continuously holding the axle at the given
 * {@link ClimberConstants.Position} setpoint.
 */
class SetPosition extends Command {

    final Climber climber;
    final ClimberConstants.Position position;

    /**
     * Creates a SetPosition command.
     *
     * @param climber  The climber subsystem
     * @param position The target axle position
     */
    public SetPosition(Climber climber, ClimberConstants.Position position) {
        this.climber = climber;
        this.position = position;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return climber.axleData().leader.pid().atTarget();
    }
}
