// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.alerter.Rumbler;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.constants.IOConstants;
import frc.robot.constants.RuntimeConstants;
import frc.robot.controller.XboxEliteController;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.axle.AxleReplay;
import frc.robot.subsystems.climber.axle.AxleSpark;
import frc.robot.subsystems.climber.cylinder.CylinderPH;
import frc.robot.subsystems.climber.cylinder.CylinderReplay;
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
        switch (RuntimeConstants.kCurrentMode) {
            case REAL -> new GyroIONavX();
            case SIM -> new GyroIO() {};
            default -> new GyroIO() {};
        },
        switch (RuntimeConstants.kCurrentMode) {
            case REAL -> moduleId -> new ModuleIOSpark(moduleId);
            case SIM -> moduleId -> new ModuleIOSim();
            default -> moduleId -> new ModuleIO() {};
        }
    );
    final Shooter shooter = new Shooter(
        switch (RuntimeConstants.kCurrentMode) {
            case REAL -> new ShooterSpark();
            case SIM -> new ShooterReplay();
            default -> new ShooterReplay();
        },
        drive
    );
    final Spindexer spindexer = new Spindexer(
        switch (RuntimeConstants.kCurrentMode) {
            case REAL -> new SpindexerSpark();
            case SIM -> new SpindexerReplay();
            default -> new SpindexerReplay();
        }
    );
    final Climber climber = switch (RuntimeConstants.kCurrentMode) {
        case REAL -> {
            PneumaticHub ph = new PneumaticHub(IOConstants.Climber.kPhId);
            yield new Climber(
                new AxleSpark(),
                new CylinderPH(
                    ph,
                    IOConstants.Climber.kInnerFwd,
                    IOConstants.Climber.kInnerRev
                ),
                new CylinderPH(
                    ph,
                    IOConstants.Climber.kOuterLeftFwd,
                    IOConstants.Climber.kOuterLeftRev
                ),
                new CylinderPH(
                    ph,
                    IOConstants.Climber.kOuterRightFwd,
                    IOConstants.Climber.kOuterRightRev
                )
            );
        }
        default -> new Climber(
            new AxleReplay(),
            new CylinderReplay(),
            new CylinderReplay(),
            new CylinderReplay()
        );
    };
    final Intake intake = new Intake(
        switch (RuntimeConstants.kCurrentMode) {
            case REAL -> new PivotSpark();
            default -> new PivotReplay();
        },
        switch (RuntimeConstants.kCurrentMode) {
            case REAL -> new RollerSpark();
            default -> new RollerReplay();
        },
        drive
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

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();
        Rumbler.getInstance().provideControllers(driverctl, operctl);
    }

    /**
     * Use this method to define your button->command mappings
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(new JoystickDrive(driverctl, drive));

        // Reset gyro to 0° when B button is pressed
        driverctl
            .start()
            .onTrue(
                Commands.runOnce(
                    // SAFETY: running this command means we see button presses, DS connected
                    () -> drive.setHeading(AllianceUtil.unsafe.zero()),
                    drive
                ).ignoringDisable(true)
            );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return selector.build(drive).cmd();
    }
}
