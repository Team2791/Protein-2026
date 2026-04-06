package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.PointAtHub;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;

/**
 * Command that points the robot at the hub and shoots.
 *
 * @see PointAtHub
 * @see Shoot
 */
public class PointAndShoot extends ParallelCommandGroup {

    public PointAndShoot(
        Drive drive,
        Spindexer spindexer,
        Shooter shooter,
        CommandXboxController ctl
    ) {
        addCommands(new PointAtHub(drive, shooter, ctl), new Shoot(spindexer));
    }
}
