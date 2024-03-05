// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2024;

import ca.warp7.frc2024.subsystems.Intake.IntakeIO;
import ca.warp7.frc2024.subsystems.Intake.IntakeIOSim;
import ca.warp7.frc2024.subsystems.Intake.IntakeIOSparkMax;
import ca.warp7.frc2024.subsystems.Intake.IntakeSubsystem;
import ca.warp7.frc2024.subsystems.arm.ArmIO;
import ca.warp7.frc2024.subsystems.arm.ArmIOSim;
import ca.warp7.frc2024.subsystems.arm.ArmIOSparkMax;
import ca.warp7.frc2024.subsystems.arm.ArmSubsystem;
import ca.warp7.frc2024.subsystems.arm.ArmSubsystem.SETPOINT;
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
import ca.warp7.frc2024.subsystems.feeder.FeederIOSparkMax;
import ca.warp7.frc2024.subsystems.feeder.FeederSubsystem;
import ca.warp7.frc2024.subsystems.shooter.ShooterModuleIO;
import ca.warp7.frc2024.subsystems.shooter.ShooterModuleIOSim;
import ca.warp7.frc2024.subsystems.shooter.ShooterModuleIOSparkMax550;
import ca.warp7.frc2024.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

// USe absolute encoder if greater than x degrees
public class RobotContainer {
    private final SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final ClimberSubsystem climberSubsystem;

    /* OI Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandXboxController technician = new CommandXboxController(2);

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
                armSubsystem = new ArmSubsystem(new ArmIOSparkMax(11, 10, 0, 1, 2, new Rotation2d(1.543)));
                intakeSubsystem = new IntakeSubsystem(new IntakeIOSparkMax(31, 4));
                shooterSubsystem = new ShooterSubsystem(
                        new ShooterModuleIOSparkMax550(22, true),
                        new ShooterModuleIOSparkMax550(23, false),
                        new ShooterModuleIOSparkMax550(21, true),
                        new ShooterModuleIOSparkMax550(20, false));
                feederSubsystem = new FeederSubsystem(new FeederIOSparkMax(24, 25, 3));
                climberSubsystem = new ClimberSubsystem(new ClimberIOSparkMaxNeo(30));
                break;
            case SIM:
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystem(
                        new GyroIO() {},
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim());
                armSubsystem = new ArmSubsystem(new ArmIOSim() {});
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

        if (!Constants.TUNING_MODE) {
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
                ClimberSubsystem.climberCommand(climberSubsystem, () -> operator.getLeftY()));

        swerveDrivetrainSubsystem.setDefaultCommand(SwerveDrivetrainSubsystem.teleopDriveCommand(
                swerveDrivetrainSubsystem,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                driver.rightBumper()));

        // operator.a().onTrue(Commands.runOnce(() -> {
        //     shooterSubsystem.runShooterRPM(topRightShooterSpeed.get(), 0);
        //     shooterSubsystem.runShooterRPM(topLeftShooterSpeed.get(), 1);
        //     shooterSubsystem.runShooterRPM(bottomLeftShooterSpeed.get(), 2);
        //     shooterSubsystem.runShooterRPM(bottomRightShooterSpeed.get(), 3);
        // }));

        operator.b().onTrue(Commands.runOnce(() -> {
            shooterSubsystem.runShooterRPM(0, 0, 1, 2, 3);
            shooterSubsystem.stopShooter();
        }));

        operator.x().onTrue(Commands.runOnce(() -> {
            armSubsystem.setSetpoint(SETPOINT.HANDOFF_INTAKE);
        }));
        operator.y().onTrue(Commands.runOnce(() -> {
            armSubsystem.setSetpoint(SETPOINT.AMP);
        }));

        /* Intake */
        Trigger armSetpointTrigger = new Trigger(() -> armSubsystem.atSetpoint(SETPOINT.HANDOFF_INTAKE));
        Trigger intakeTrigger = new Trigger(() -> intakeSubsystem.getSensor()).debounce(0.075);

        operator.rightTrigger()
                .and(armSetpointTrigger)
                .onTrue(intakeSubsystem.runVolts(10).until(() -> intakeTrigger.getAsBoolean()));

        intakeTrigger
                .onTrue(feederSubsystem.runVolts(3).until(() -> feederSubsystem.getSensor()))
                .onTrue(intakeSubsystem.runVolts(6).until(() -> feederSubsystem.getSensor()));

        Command shooter = Commands.runOnce(() -> {
            shooterSubsystem.runShooterRPM(topRightShooterSpeed.get(), 0);
            shooterSubsystem.runShooterRPM(topLeftShooterSpeed.get(), 1);
            shooterSubsystem.runShooterRPM(bottomLeftShooterSpeed.get(), 2);
            shooterSubsystem.runShooterRPM(bottomRightShooterSpeed.get(), 3);
        });

        operator.leftBumper().onTrue(Commands.runOnce(() -> shooterSubsystem.runShooterRPM(7000, 0, 1, 2, 3)));

        operator.a()
                .onTrue(Commands.sequence(
                        feederSubsystem.runVolts(-2).withTimeout(0.125),
                        shooter,
                        new WaitCommand(2.5),
                        feederSubsystem.runVolts(11).withTimeout(1),
                        new WaitCommand(1),
                        Commands.runOnce(() -> shooterSubsystem.stopShooter())));
    }

    public Command getAutonomousCommand() {
        return autonomousRoutineChooser.get();
    }
}
