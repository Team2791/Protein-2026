package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.spindexer.Spindexer;

/**
 * Command that loads balls into the shooter by running the spindexer.
 *
 * <p>
 * The shooter is always-running at the correct speed, so this command only
 * controls the spindexer.
 */
public class Shoot extends RepeatCommand {

    static class RunSpindexer extends Command {

        final Spindexer spindexer;
        final Timer timer = new Timer();

        RunSpindexer(Spindexer spindexer) {
            this.spindexer = spindexer;
            addRequirements(spindexer);
        }

        @Override
        public void initialize() {
            spindexer.set(true);
            timer.restart();
        }

        @Override
        public void end(boolean interrupted) {
            spindexer.set(false);
        }

        @Override
        public boolean isFinished() {
            return (
                timer.hasElapsed(ShooterConstants.Shoot.kStallTimeout) &&
                spindexer.data().spindexer.velocity() <
                ShooterConstants.Shoot.kStallVelocity
            );
        }
    }

    static class Reverse extends Command {

        final Spindexer spindexer;

        Reverse(Spindexer spindexer) {
            this.spindexer = spindexer;
        }

        @Override
        public void initialize() {
            spindexer.reverse();
        }

        @Override
        public void end(boolean interrupted) {
            spindexer.set(false);
        }
    }

    public static class ReverseTimed extends ParallelDeadlineGroup {

        public ReverseTimed(Spindexer spindexer) {
            super(
                new WaitCommand(ShooterConstants.Shoot.kReverseDuration),
                new Reverse(spindexer)
            );
        }
    }

    public Shoot(Spindexer spindexer) {
        super(
            new SequentialCommandGroup(
                new RunSpindexer(spindexer),
                new ReverseTimed(spindexer)
            )
        );
    }
}
