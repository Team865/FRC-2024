// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2024;

import ca.warp7.frc2024.subsystems.Intake.IntakeIO;
import ca.warp7.frc2024.subsystems.Intake.IntakeIOSim;
import ca.warp7.frc2024.subsystems.Intake.IntakeSubsystem;
import ca.warp7.frc2024.subsystems.arm.ArmIO;
import ca.warp7.frc2024.subsystems.arm.ArmIOSim;
import ca.warp7.frc2024.subsystems.arm.ArmSubsystem;
import ca.warp7.frc2024.subsystems.climber.ClimberIO;
import ca.warp7.frc2024.subsystems.climber.ClimberIOSim;
import ca.warp7.frc2024.subsystems.climber.ClimberIOSparkMaxNeo;
import ca.warp7.frc2024.subsystems.climber.ClimberSubsystem;
import ca.warp7.frc2024.subsystems.drivetrain.GyroIO;
import ca.warp7.frc2024.subsystems.drivetrain.GyroIONavX;
import ca.warp7.frc2024.subsystems.drivetrain.SwerveDrivetrainSubsystem;
import ca.warp7.frc2024.subsystems.drivetrain.SwerveModuleIO;
import ca.warp7.frc2024.subsystems.drivetrain.SwerveModuleIOFalcon500;
import ca.warp7.frc2024.subsystems.drivetrain.SwerveModuleIOSim;
import ca.warp7.frc2024.subsystems.feeder.FeederIO;
import ca.warp7.frc2024.subsystems.feeder.FeederIOSim;
import ca.warp7.frc2024.subsystems.feeder.FeederIOSparkMax550;
import ca.warp7.frc2024.subsystems.feeder.FeederSubsystem;
import ca.warp7.frc2024.subsystems.shooter.ShooterModuleIO;
import ca.warp7.frc2024.subsystems.shooter.ShooterModuleIOSim;
import ca.warp7.frc2024.subsystems.shooter.ShooterModuleIOSparkMax550;
import ca.warp7.frc2024.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class RobotContainer {
    private final SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final ClimberSubsystem climberSubsystem;

    private final CommandXboxController driverController = new CommandXboxController(0);

    private final LoggedDashboardNumber shooterSpeed = new LoggedDashboardNumber("Shooter Speed", 1500.0);

    private final LoggedDashboardChooser<Command> autonomousRoutineChooser;

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystem(
                        new GyroIONavX() {},
                        new SwerveModuleIOFalcon500(12, 11, 10, new Rotation2d(-0.1)),
                        new SwerveModuleIOFalcon500(22, 21, 20, new Rotation2d(-1.379)),
                        new SwerveModuleIOFalcon500(32, 31, 30, new Rotation2d(-3.102)),
                        new SwerveModuleIOFalcon500(42, 41, 40, new Rotation2d(-1.379)));
                armSubsystem = new ArmSubsystem(new ArmIO() {});
                intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});
                shooterSubsystem = new ShooterSubsystem(
                        new ShooterModuleIOSparkMax550(22, true),
                        new ShooterModuleIOSparkMax550(23, false),
                        new ShooterModuleIOSparkMax550(21, true),
                        new ShooterModuleIOSparkMax550(20, false));
                feederSubsystem = new FeederSubsystem(new FeederIOSparkMax550(24, 25));
                climberSubsystem = new ClimberSubsystem(new ClimberIOSparkMaxNeo(30));
                break;
            case SIM:
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystem(
                        new GyroIO() {},
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim());
                armSubsystem = new ArmSubsystem(new ArmIOSim());
                intakeSubsystem = new IntakeSubsystem(new IntakeIOSim() {});
                shooterSubsystem = new ShooterSubsystem(
                        new ShooterModuleIOSim(),
                        new ShooterModuleIOSim(),
                        new ShooterModuleIOSim(),
                        new ShooterModuleIOSim());
                feederSubsystem = new FeederSubsystem(new FeederIOSim());
                climberSubsystem = new ClimberSubsystem(new ClimberIOSim());

                break;
            default:
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystem(
                        new GyroIO() {},
                        new SwerveModuleIO() {},
                        new SwerveModuleIO() {},
                        new SwerveModuleIO() {},
                        new SwerveModuleIO() {});
                armSubsystem = new ArmSubsystem(new ArmIO() {});
                intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});
                shooterSubsystem = new ShooterSubsystem(
                        new ShooterModuleIO() {},
                        new ShooterModuleIO() {},
                        new ShooterModuleIO() {},
                        new ShooterModuleIO() {});
                feederSubsystem = new FeederSubsystem(new FeederIO() {});
                climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        }

        autonomousRoutineChooser = new LoggedDashboardChooser<>("Autonomous Routine Chooser");

        autonomousRoutineChooser.addOption("Zero shooter", Commands.runOnce(() -> shooterSubsystem.zeroEncoder()));

        autonomousRoutineChooser.addOption(
                "Shooter quasistatic forward", shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

        autonomousRoutineChooser.addOption(
                "Shooter quasistatic reverse", shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        autonomousRoutineChooser.addOption(
                "Shooter dynamic forward", shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));

        autonomousRoutineChooser.addOption(
                "Shooter dynamic reverse", shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        configureBindings();
    }

    private void configureBindings() {
        // climberSubsystem.setDefaultCommand(
        //         ClimberSubsystem.climberCommand(climberSubsystem, () -> driverController.getLeftY()));

        swerveDrivetrainSubsystem.setDefaultCommand(SwerveDrivetrainSubsystem.teleopDriveCommand(
                swerveDrivetrainSubsystem,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                driverController.rightBumper()));

        // driverController.a().onTrue(Commands.runOnce(() -> {
        //     armSubsystem.setSetpoint(Rotation2d.fromDegrees(130));
        // }));
        // driverController.b().onTrue(Commands.runOnce(() -> {
        //     armSubsystem.setSetpoint(Rotation2d.fromDegrees(200));
        // }));
        // driverController.x().onTrue(intakeSubsystem.setIntakeVoltage(5));
        // driverController.x().onFalse(intakeSubsystem.setIntakeVoltage(0));

        driverController.a().onTrue(Commands.runOnce(() -> {
            shooterSubsystem.setShooterRPM(shooterSpeed.get(), 0);
            shooterSubsystem.setShooterRPM(shooterSpeed.get(), 1);

            shooterSubsystem.setShooterRPM(shooterSpeed.get(), 3);

            shooterSubsystem.setShooterRPM(shooterSpeed.get(), 2);
        }));

        driverController.b().onTrue(Commands.runOnce(() -> {
            shooterSubsystem.setShooterRPM(0, 0);
            shooterSubsystem.setShooterRPM(0, 1);
            shooterSubsystem.setShooterRPM(0, 2);
            shooterSubsystem.setShooterRPM(0, 3);
            shooterSubsystem.stopShooter();
        }));

        driverController.leftTrigger().onTrue(Commands.runOnce(() -> {
            feederSubsystem.setVolts(-6);
        }));

        driverController.rightBumper().onTrue(Commands.runOnce(() -> {
            feederSubsystem.setVolts(6);
        }));

        driverController.leftBumper().onTrue(Commands.runOnce(() -> {
            feederSubsystem.setVolts(0);
        }));
    }

    public Command getAutonomousCommand() {

        // return Commands.runOnce(() -> System.out.println("Auto"));
        return autonomousRoutineChooser.get();
    }
}
