# Protein-2026

Protein is Team 2791's 2026 FRC robot code. Built on the AdvantageKit logging framework, it utilizes a hardware-abstracted "split-IO" architecture to ensure consistency across real hardware, simulation, and log replay.

## Robot Overview

The robot features a high-performance swerve drivetrain coupled with a precision shooting mechanism and an automated intake system.

### Core Subsystems

*   **Drivetrain**: 4-module SDS Swerve drive using SparkFlex motor controllers. Includes field-centric control, automatic alliance flipping, and Meta 3S (QuestNav) tracking for robust localization.
*   **Shooter**: Distance-aware flywheel mechanism. Uses a quadratic regression to automatically calculate optimal wheel velocity based on the robot's distance to the hub.
*   **Intake**: Pivot-based intake system. Features an "auto-deploy" mode that extends the intake as soon as the robot is enabled to maximize collection uptime.
*   **Spindexer**: Intelligent feeding system that synchronizes ball delivery with the shooter flywheel's ready state.

## Architecture

This project uses the **AdvantageKit** framework. All subsystem logic is separated from hardware interaction via an IO interface pattern:
*   `Subsystem.java`: High-level logic and state management.
*   `SubsystemIO.java`: Abstract interface for hardware interaction.
*   `SubsystemIOSpark.java`: Real hardware implementation using REVLib.
*   `SubsystemIOReplay.java`: No-op implementation for log playback.

## Getting Started

### Development Commands
*   **Build**: `./gradlew build`
*   **Simulate**: Run the "Simulate Robot" task in VS Code or `./gradlew simulateJava`
*   **Deploy**: `./gradlew deploy`

## Specialized Technologies

*   **QuestNav (Meta 3S)**: Integrated tracking using a Meta headset for high-fidelity pose estimation and gyro drift compensation.
*   **Elastic Dashboard**: Custom notification and telemetry system for real-time driver feedback.
*   **Automatic Regression**: Live calculation of shooter powers using `ShooterConstants.Regression`.

---
*Developed with ❤️ by Team 2791 Shaker Robotics.*
