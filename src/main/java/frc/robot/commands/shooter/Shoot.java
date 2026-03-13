package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.Spindexer;

/**
 * Command that loads balls into the shooter by running the spindexer.
 *
 * <p>
 * The shooter is always-running at the correct speed, so this command only
 * controls the spindexer.
 */
class Shoot extends Command {

    final Spindexer spindexer;

    public Shoot(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        spindexer.set(true);
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.set(false);
    }
}
