// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2024;

import ca.warp7.frc2024.subsystems.Intake.IntakeIO;
import ca.warp7.frc2024.subsystems.Intake.IntakeIOSim;
import ca.warp7.frc2024.subsystems.Intake.IntakeIOSparkMaxNeo;
import ca.warp7.frc2024.subsystems.Intake.IntakeSubsystem;
import ca.warp7.frc2024.subsystems.arm.ArmIO;
import ca.warp7.frc2024.subsystems.arm.ArmIOSim;
import ca.warp7.frc2024.subsystems.arm.ArmIOSparkMaxNeo;
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
import edu.wpi.first.wpilibj.DriverStation;
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

    private final CommandXboxController primaryController = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);
    private final CommandXboxController technicianController = new CommandXboxController(2);

    private final LoggedDashboardNumber topRightShooterSpeed =
            new LoggedDashboardNumber("Top Right Shooter Speed", 1500.0);
    private final LoggedDashboardNumber topLeftShooterSpeed =
            new LoggedDashboardNumber("Top Left Shooter Speed", 1500.0);
    private final LoggedDashboardNumber bottomLeftShooterSpeed =
            new LoggedDashboardNumber("Bottom Left Shooter Speed", 1500.0);
    private final LoggedDashboardNumber bottomRightShooterSpeed =
            new LoggedDashboardNumber("Bottom Right Shooter Speed", 1500.0);

    private final LoggedDashboardNumber feederVolts = new LoggedDashboardNumber("Feeder Volts", -6);
    private final LoggedDashboardNumber reverseFeederVolts = new LoggedDashboardNumber("Reverse Feeder Volts", 6);

    private final LoggedDashboardNumber intakeVolts = new LoggedDashboardNumber("Intake Volts", -6);

    private final LoggedDashboardNumber highPoint = new LoggedDashboardNumber("High Point", 35);
    private final LoggedDashboardNumber lowPoint = new LoggedDashboardNumber("Low Point", 78);

    private final LoggedDashboardChooser<Command> autonomousRoutineChooser;

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystem(
                        new GyroIONavX() {},
                        new SwerveModuleIOFalcon500(12, 11, 10, Rotation2d.fromRotations(0.488)),
                        new SwerveModuleIOFalcon500(22, 21, 20, Rotation2d.fromRotations(-0.242)),
                        new SwerveModuleIOFalcon500(32, 31, 30, Rotation2d.fromRotations(0.096)),
                        new SwerveModuleIOFalcon500(42, 41, 40, Rotation2d.fromRotations(0.008)));
                armSubsystem = new ArmSubsystem(new ArmIOSparkMaxNeo(10, 11));
                intakeSubsystem = new IntakeSubsystem(new IntakeIOSparkMaxNeo(31));
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

        if (!Constants.COMPETITION_DEPLOYMENT) {
            autonomousRoutineChooser.addOption(
                    "Shooter quasistatic forward", shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

            autonomousRoutineChooser.addOption(
                    "Shooter quasistatic reverse", shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            autonomousRoutineChooser.addOption(
                    "Shooter dynamic forward", shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));

            autonomousRoutineChooser.addOption(
                    "Shooter dynamic reverse", shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
            autonomousRoutineChooser.addOption("Zero shooter", Commands.runOnce(() -> {
                shooterSubsystem.zeroEncoder();
            }));
        }

        configureBindings();

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureBindings() {
        climberSubsystem.setDefaultCommand(
                ClimberSubsystem.climberCommand(climberSubsystem, () -> secondaryController.getLeftY()));

        swerveDrivetrainSubsystem.setDefaultCommand(SwerveDrivetrainSubsystem.teleopDriveCommand(
                swerveDrivetrainSubsystem,
                () -> -primaryController.getLeftY(),
                () -> -primaryController.getLeftX(),
                () -> -primaryController.getRightX(),
                primaryController.rightBumper()));

        secondaryController.a().onTrue(Commands.runOnce(() -> {
            shooterSubsystem.runShooterRPM(topRightShooterSpeed.get(), 0);
            shooterSubsystem.runShooterRPM(topLeftShooterSpeed.get(), 1);
            shooterSubsystem.runShooterRPM(bottomLeftShooterSpeed.get(), 2);
            shooterSubsystem.runShooterRPM(bottomRightShooterSpeed.get(), 3);
        }));

        secondaryController.b().onTrue(Commands.runOnce(() -> {
            shooterSubsystem.runShooterRPM(0, 0, 1, 2, 3);
            shooterSubsystem.stopShooter();
        }));

        secondaryController.x().onTrue(Commands.runOnce(() -> {
            armSubsystem.runSetpoint(Rotation2d.fromDegrees(lowPoint.get()));
        }));
        secondaryController.y().onTrue(Commands.runOnce(() -> {
            armSubsystem.runSetpoint(Rotation2d.fromDegrees(highPoint.get()));
        }));

        secondaryController
                .rightBumper()
                .onTrue(Commands.runOnce(() -> {
                    feederSubsystem.setVolts(feederVolts.get());
                }))
                .onFalse(Commands.runOnce(() -> {
                    feederSubsystem.setVolts(0);
                }));

        secondaryController
                .leftBumper()
                .onTrue(Commands.runOnce(() -> {
                    intakeSubsystem.runVolts(intakeVolts.get());
                }))
                .onFalse(Commands.runOnce(() -> {
                    intakeSubsystem.runVolts(0);
                }));

        secondaryController
                .rightTrigger()
                .onTrue(Commands.runOnce(() -> {
                    feederSubsystem.setVolts(reverseFeederVolts.get());
                }))
                .onFalse(Commands.runOnce(() -> {
                    feederSubsystem.setVolts(0);
                }));
    }

    public Command getAutonomousCommand() {
        return autonomousRoutineChooser.get();
    }
}
