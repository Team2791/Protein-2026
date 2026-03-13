package frc.robot.commands.climber;

import static frc.robot.constants.ClimberConstants.Position.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;

/**
 * Descend from an autonomous L1 climb.
 *
 * <p>
 * Reverses a {@link ClimbAuto} by expanding (undoing the partial contraction),
 * fully contracting, then disengaging the inner hooks. Sequence:
 *
 * <ol>
 *   <li>Expand axle (undo partial contraction)
 *   <li>Fully contract axle
 *   <li>Disengage all hooks
 * </ol>
 */
public class ClimbDown extends SequentialCommandGroup {

    public ClimbDown(Climber climber, Drive drive, Intake intake) {
        addCommands(
            new SetPosition(climber, EXPAND),
            new Command() {
                final Timer timer = new Timer();

                {
                    addRequirements(drive);
                }

                @Override
                public void initialize() {
                    drive.runVelocity(
                        new ChassisSpeeds(
                            0.0,
                            ClimberConstants.kReverseSpeed,
                            0.0
                        )
                    );

                    timer.restart();
                }

                @Override
                public void end(boolean interrupted) {
                    drive.runVelocity(new ChassisSpeeds());
                    timer.stop();
                }

                @Override
                public boolean isFinished() {
                    return timer.get() >= ClimberConstants.kReverseDuration;
                }
            },
            new SetPosition(climber, CONTRACT),
            new Engage(climber, intake, false, false)
        );
    }
}
