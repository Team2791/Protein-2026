package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.alerter.Rumbler;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.commands.drive.PointAtHub;
import frc.robot.commands.drive.SysId;
import frc.robot.commands.intake.Deploy;
import frc.robot.commands.intake.Roll;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.constants.IOConstants;
import frc.robot.constants.IntakeConstants.Roller.RollerState;
import frc.robot.constants.ShooterConstants;
import frc.robot.controller.XboxEliteController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.PivotReplay;
import frc.robot.subsystems.intake.pivot.PivotSpark;
import frc.robot.subsystems.intake.roller.RollerReplay;
import frc.robot.subsystems.intake.roller.RollerSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterReplay;
import frc.robot.subsystems.shooter.ShooterSpark;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerReplay;
import frc.robot.subsystems.spindexer.SpindexerSpark;
import frc.robot.util.AdvantageUtil;
import frc.robot.util.AllianceUtil;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    final Drive drive = new Drive(
        AdvantageUtil.match(GyroIONavX::new, () -> new GyroIO() {}),
        AdvantageUtil.match(
            ModuleIOSpark::new,
            moduleId -> new ModuleIOSim(),
            moduleId -> new ModuleIO() {}
        )
    );

    final Shooter shooter = new Shooter(
        AdvantageUtil.match(ShooterSpark::new, ShooterReplay::new),
        drive
    );

    final Spindexer spindexer = new Spindexer(
        AdvantageUtil.match(SpindexerSpark::new, SpindexerReplay::new)
    );

    // final Climber climber = AdvantageUtil.matchReal(
    //     () -> {
    //         PneumaticHub ph = new PneumaticHub(IOConstants.Climber.kPhId);
    //         return new Climber(new AxleSpark(), ch -> new CylinderPH(ph, ch));
    //     },
    //     () -> new Climber(new AxleReplay(), ch -> new CylinderReplay())
    // );

    final Intake intake = new Intake(
        AdvantageUtil.match(() -> new PivotSpark(), () -> new PivotReplay()),
        AdvantageUtil.match(() -> new RollerSpark(), () -> new RollerReplay())
    );

    // Controllers
    final XboxEliteController driverctl = new XboxEliteController(
        IOConstants.Controller.kDriver,
        IOConstants.Controller.kDriverPaddles
    );
    final XboxEliteController operctl = new XboxEliteController(
        IOConstants.Controller.kOperator,
        IOConstants.Controller.kOperatorPaddles
    );

    // Auto selector
    final AutoSelector selector = new AutoSelector(drive);
    final SendableChooser<Command> sysid = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();
        configureSysId();
        Rumbler.getInstance().provideControllers(driverctl, operctl);

        SmartDashboard.putData("AutoSysId", sysid);
    }

    /**
     * Use this method to define your button->command mappings
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(new JoystickDrive(driverctl, drive));

        // Reset gyro to 0° when start button is pressed
        driverctl
            .start()
            .onTrue(
                Commands.runOnce(
                    // SAFETY: running this command means we see button presses, DS connected
                    () -> drive.setHeading(AllianceUtil.unsafe.zero()),
                    drive
                ).ignoringDisable(true)
            );

        // Driver: lock
        driverctl.leftBumper().whileTrue(Commands.run(drive::lockX, drive));

        // Driver: point and shoot
        driverctl.rightBumper().whileTrue(new Shoot(spindexer));
        driverctl.rightBumper().whileFalse(new Shoot.ReverseTimed(spindexer));
        driverctl.rightTrigger().whileTrue(new PointAtHub(drive, driverctl));

        driverctl
            .y()
            .onTrue(new SetShooter(shooter, ShooterConstants.Setpoint.kFar));

        driverctl
            .x()
            .onTrue(new SetShooter(shooter, ShooterConstants.Setpoint.kMedium));

        driverctl
            .a()
            .onTrue(new SetShooter(shooter, ShooterConstants.Setpoint.kNear));

        driverctl
            .b()
            .onTrue(new SetShooter(shooter, ShooterConstants.Setpoint.kLow));

        operctl.a().onTrue(new Deploy(intake, false));
        operctl.a().onFalse(new Deploy(intake, true));
        operctl.rightBumper().whileTrue(new Roll(intake, RollerState.kStopped));
        operctl.leftBumper().whileTrue(new Roll(intake, RollerState.kReverse));
    }

    private void configureSysId() {
        sysid.addOption(
            "Drive Wheel Radius Characterization",
            SysId.wheelRadiusCharacterization(drive)
        );
        sysid.addOption(
            "Drive Simple FF Characterization",
            SysId.feedforwardCharacterization(drive)
        );
        sysid.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        sysid.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        sysid.addOption(
            "Drive SysId (Dynamic Forward)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        sysid.addOption(
            "Drive SysId (Dynamic Reverse)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );

        SmartDashboard.putData("SysIdAuto", sysid);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return selector.build(drive, shooter, spindexer);
    }
}
