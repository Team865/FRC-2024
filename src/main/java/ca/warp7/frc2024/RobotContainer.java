// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2024;

import ca.warp7.frc2024.subsystems.drivetrain.GyroIO;
import ca.warp7.frc2024.subsystems.drivetrain.SwerveDrivetrainSubsystem;
import ca.warp7.frc2024.subsystems.drivetrain.SwerveModuleIO;
import ca.warp7.frc2024.subsystems.drivetrain.SwerveModuleIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;

    private final CommandXboxController driverController = new CommandXboxController(0);

    // private final LoggedDashboardChooser<Command> autonomousRoutineChooser;

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
            case SIM:
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystem(
                        new GyroIO() {},
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim());
                break;
            default:
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystem(
                        new GyroIO() {},
                        new SwerveModuleIO() {},
                        new SwerveModuleIO() {},
                        new SwerveModuleIO() {},
                        new SwerveModuleIO() {});
                break;
        }

        configureBindings();
    }

    private void configureBindings() {
        swerveDrivetrainSubsystem.setDefaultCommand(SwerveDrivetrainSubsystem.teleopDriveCommand(
                swerveDrivetrainSubsystem,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                driverController.rightBumper()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
