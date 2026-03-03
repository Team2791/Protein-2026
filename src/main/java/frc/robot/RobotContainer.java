// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.alerter.Rumbler;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.constants.IOConstants;
import frc.robot.constants.RuntimeConstants;
import frc.robot.controller.XboxEliteController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.util.AllianceUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    private final Drive drive = new Drive(
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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
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
