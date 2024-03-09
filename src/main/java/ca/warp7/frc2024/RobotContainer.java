// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2024;

import ca.warp7.frc2024.Constants.CLIMBER.STATE;
import ca.warp7.frc2024.subsystems.Intake.IntakeIO;
import ca.warp7.frc2024.subsystems.Intake.IntakeIOSim;
import ca.warp7.frc2024.subsystems.Intake.IntakeIOSparkMax;
import ca.warp7.frc2024.subsystems.Intake.IntakeSubsystem;
import ca.warp7.frc2024.subsystems.arm.ArmIO;
import ca.warp7.frc2024.subsystems.arm.ArmIOSim;
import ca.warp7.frc2024.subsystems.arm.ArmIOSparkMax;
import ca.warp7.frc2024.subsystems.arm.ArmSubsystem;
import ca.warp7.frc2024.subsystems.arm.ArmSubsystem.Setpoint;
import ca.warp7.frc2024.subsystems.climber.ClimberIO;
import ca.warp7.frc2024.subsystems.climber.ClimberIOSim;
import ca.warp7.frc2024.subsystems.climber.ClimberIOSparkMaxNeo;
import ca.warp7.frc2024.subsystems.climber.ClimberSubsystem;
import ca.warp7.frc2024.subsystems.drivetrain.GyroIO;
import ca.warp7.frc2024.subsystems.drivetrain.GyroIONavX;
import ca.warp7.frc2024.subsystems.drivetrain.SwerveDrivetrainSubsystem;
import ca.warp7.frc2024.subsystems.drivetrain.SwerveDrivetrainSubsystem.PointAtLocation;
import ca.warp7.frc2024.subsystems.drivetrain.SwerveModuleIO;
import ca.warp7.frc2024.subsystems.drivetrain.SwerveModuleIOFalcon500;
import ca.warp7.frc2024.subsystems.drivetrain.SwerveModuleIOSim;
import ca.warp7.frc2024.subsystems.feeder.FeederIO;
import ca.warp7.frc2024.subsystems.feeder.FeederIOSim;
import ca.warp7.frc2024.subsystems.feeder.FeederIOSparkMax;
import ca.warp7.frc2024.subsystems.feeder.FeederSubsystem;
import ca.warp7.frc2024.subsystems.leds.LEDSubsystem;
import ca.warp7.frc2024.subsystems.leds.LEDSubsystem.SparkColor;
import ca.warp7.frc2024.subsystems.shooter.ShooterModuleIO;
import ca.warp7.frc2024.subsystems.shooter.ShooterModuleIOSim;
import ca.warp7.frc2024.subsystems.shooter.ShooterModuleIOSparkMax550;
import ca.warp7.frc2024.subsystems.shooter.ShooterSubsystem;
import ca.warp7.frc2024.subsystems.vision.VisionIO;
import ca.warp7.frc2024.subsystems.vision.VisionIOLimelight;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private final LEDSubsystem ledSubsystem;

    /* OI Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandXboxController technician = new CommandXboxController(2);

    private final LoggedDashboardNumber topRightShooterSpeed =
            new LoggedDashboardNumber("Top Right Shooter Speed", 9000.0);
    private final LoggedDashboardNumber topLeftShooterSpeed =
            new LoggedDashboardNumber("Top Left Shooter Speed", 9000.0);
    private final LoggedDashboardNumber bottomLeftShooterSpeed =
            new LoggedDashboardNumber("Bottom Left Shooter Speed", 9000.0);
    private final LoggedDashboardNumber bottomRightShooterSpeed =
            new LoggedDashboardNumber("Bottom Right Shooter Speed", 9000.0);

    private final LoggedDashboardChooser<Command> autonomousRoutineChooser;

    /* Commands */
    private final Command simpleIntake;
    private final Command simpleShoot;
    private final Command simpleAmp;
    private final Command noteFlowForward;
    private final Command noteFlowReverse;
    private final Command stopNoteFlow;

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystem(
                        new GyroIONavX() {},
                        new VisionIOLimelight("limelight-front"),
                        new VisionIOLimelight("limelight-rear"),
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
                ledSubsystem = new LEDSubsystem(0);
                break;
            case SIM:
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystem(
                        new GyroIO() {},
                        new VisionIO() {},
                        new VisionIO() {},
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
                ledSubsystem = new LEDSubsystem(0);

                break;
            default:
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystem(
                        new GyroIO() {},
                        new VisionIO() {},
                        new VisionIO() {},
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
                ledSubsystem = new LEDSubsystem(0);
        }

        simpleIntake = Commands.parallel(
                        ledSubsystem.solidColorCommand(SparkColor.SKY_BLUE),
                        intakeSubsystem.runVoltage(10).until(intakeSubsystem.sensorTrigger()),
                        feederSubsystem.runVoltage(8).until(intakeSubsystem.sensorTrigger()))
                .andThen(
                        Commands.parallel(
                                shooterSubsystem.runRPMCommand(-1500, 0, 1, 2, 3),
                                ledSubsystem.blinkColorCommand(SparkColor.GREEN, 0.25, 1),
                                intakeSubsystem.runVoltage(10).until(feederSubsystem.sensorTrigger()),
                                feederSubsystem.runVoltage(8).until(feederSubsystem.sensorTrigger())),
                        shooterSubsystem.runRPMCommand(0, 0, 1, 2, 3));

        simpleShoot = Commands.sequence(
                shooterSubsystem.runRPMCommand(-8000, 0, 1, 2, 3).withTimeout(0.75),
                feederSubsystem.runVoltage(-12).withTimeout(0.1),
                shooterSubsystem.runRPMCommand(8500, 0, 1, 2, 3),
                Commands.waitSeconds(0.75),
                feederSubsystem.runVoltage(12).withTimeout(1),
                shooterSubsystem.stopShooterCommand());

        simpleAmp = Commands.sequence(
                Commands.parallel(shooterSubsystem.runRPMCommand(-5000, 0, 1, 2, 3), feederSubsystem.runVoltage(-8))
                        .withTimeout(3),
                Commands.parallel(shooterSubsystem.stopShooterCommand(), feederSubsystem.runVoltage(0))
                        .withTimeout(0.125));

        noteFlowForward = Commands.parallel(
                intakeSubsystem.runVoltage(8),
                feederSubsystem.runVoltage(8),
                shooterSubsystem.runRPMCommand(6000, 0, 1, 2, 3));

        noteFlowReverse = Commands.parallel(
                intakeSubsystem.runVoltage(-11),
                feederSubsystem.runVoltage(-8),
                shooterSubsystem.runRPMCommand(-6000, 0, 1, 2, 3));

        stopNoteFlow = Commands.parallel(
                intakeSubsystem.runVoltage(0), feederSubsystem.runVoltage(0), shooterSubsystem.stopShooterCommand());

        NamedCommands.registerCommand(
                "autoIntake",
                Commands.parallel(
                                intakeSubsystem.runVoltage(10).until(intakeSubsystem.sensorTrigger()),
                                feederSubsystem.runVoltage(8).until(intakeSubsystem.sensorTrigger()))
                        .andThen(
                                Commands.parallel(
                                        shooterSubsystem.runRPMCommand(-1500, 0, 1, 2, 3),
                                        ledSubsystem.blinkColorCommand(SparkColor.GREEN, 0.25, 1),
                                        intakeSubsystem.runVoltage(10).until(feederSubsystem.sensorTrigger()),
                                        feederSubsystem.runVoltage(8).until(feederSubsystem.sensorTrigger())),
                                shooterSubsystem.runRPMCommand(0, 0, 1, 2, 3)));
        NamedCommands.registerCommand(
                "armSubwoofer",
                armSubsystem
                        .setSetpointCommand(Setpoint.SUBWOOFER)
                        .until(armSubsystem.atSetpointTrigger(Setpoint.SUBWOOFER)));
        NamedCommands.registerCommand(
                "armStow",
                armSubsystem
                        .setSetpointCommand(Setpoint.HANDOFF_INTAKE)
                        .until(armSubsystem.atSetpointTrigger(Setpoint.HANDOFF_INTAKE)));
        NamedCommands.registerCommand(
                "shoot",
                Commands.sequence(
                        shooterSubsystem.runRPMCommand(-5000, 0, 1, 2, 3).withTimeout(0.25),
                        feederSubsystem.runVoltage(-12).withTimeout(0.0625),
                        shooterSubsystem.runRPMCommand(topRightShooterSpeed.get(), 0),
                        shooterSubsystem.runRPMCommand(topLeftShooterSpeed.get(), 1),
                        shooterSubsystem.runRPMCommand(bottomLeftShooterSpeed.get(), 2),
                        shooterSubsystem.runRPMCommand(bottomRightShooterSpeed.get(), 3),
                        Commands.waitSeconds(1),
                        feederSubsystem.runVoltage(12).withTimeout(1),
                        shooterSubsystem.stopShooterCommand()));

        Command shoot = Commands.sequence(
                shooterSubsystem.runRPMCommand(-5000, 0, 1, 2, 3).withTimeout(0.25),
                feederSubsystem.runVoltage(-12).withTimeout(0.0625),
                shooterSubsystem.runRPMCommand(topRightShooterSpeed.get(), 0),
                shooterSubsystem.runRPMCommand(topLeftShooterSpeed.get(), 1),
                shooterSubsystem.runRPMCommand(bottomLeftShooterSpeed.get(), 2),
                shooterSubsystem.runRPMCommand(bottomRightShooterSpeed.get(), 3),
                Commands.waitSeconds(1),
                feederSubsystem.runVoltage(12).withTimeout(1),
                shooterSubsystem.stopShooterCommand());

        Command armStow = armSubsystem
                .setSetpointCommand(Setpoint.HANDOFF_INTAKE)
                .until(armSubsystem.atSetpointTrigger(Setpoint.HANDOFF_INTAKE));

        Command armSubwoofer = armSubsystem
                .setSetpointCommand(Setpoint.SUBWOOFER)
                .until(armSubsystem.atSetpointTrigger(Setpoint.SUBWOOFER));

        NamedCommands.registerCommand("autoShoot", Commands.sequence(armSubwoofer, shoot, armStow));

        autonomousRoutineChooser =
                new LoggedDashboardChooser<>("Autonomous Routine Chooser", AutoBuilder.buildAutoChooser());

        if (Constants.TUNING_MODE) {
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

        configureDriverBindings();
        configureOperatorBindings();

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureDriverBindings() {

        swerveDrivetrainSubsystem.setDefaultCommand(SwerveDrivetrainSubsystem.teleopDriveCommand(
                swerveDrivetrainSubsystem,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                driver.rightBumper()));

        driver.start().onTrue(swerveDrivetrainSubsystem.zeroGyroCommand());
        driver.rightStick().onTrue(swerveDrivetrainSubsystem.zeroGyroAndPoseCommand());
        driver.leftBumper().whileTrue(armSubsystem.setSetpointCommand(Setpoint.HANDOFF_INTAKE));

        driver.a()
                .onTrue(swerveDrivetrainSubsystem.setPointAtCommand(PointAtLocation.SPEAKER))
                .onFalse(swerveDrivetrainSubsystem.setPointAtCommand(PointAtLocation.NONE));

        driver.b().onTrue(swerveDrivetrainSubsystem.stopWithXCommand());
    }

    private void configureOperatorBindings() {
        // spotless:off
        /* Intaking */
        operator.rightTrigger().and(armSubsystem.atSetpointTrigger(Setpoint.HANDOFF_INTAKE)).onTrue(simpleIntake);

        /* Arm */
        operator.povDown().onTrue(armSubsystem.setSetpointCommand(Setpoint.HANDOFF_INTAKE));
        operator.povUp().onTrue(armSubsystem.setSetpointCommand(Setpoint.PODIUM));
        operator.povRight().onTrue(armSubsystem.setSetpointCommand(Setpoint.SUBWOOFER));
        operator.povLeft().onTrue(armSubsystem.setSetpointCommand(Setpoint.AMP));

        /* Scoring */
        operator.a().and(armSubsystem.atSetpointTrigger(Setpoint.PODIUM)).onTrue(simpleShoot);
        operator.a().and(armSubsystem.atSetpointTrigger(Setpoint.SUBWOOFER)).onTrue(simpleShoot);
        operator.a().and(armSubsystem.atSetpointTrigger(Setpoint.AMP)).onTrue(simpleAmp);
//fix amp score

        /* Override Procedures */
        operator.leftBumper().onTrue(noteFlowReverse).onFalse(stopNoteFlow);
        operator.rightBumper().onTrue(noteFlowForward).onFalse(stopNoteFlow);
        operator.b().onTrue(stopNoteFlow);

        /* Climbing */
        climberSubsystem.setDefaultCommand(
                ClimberSubsystem.climberCommand(climberSubsystem, () -> operator.getLeftY()));
        operator.leftStick().onTrue(climberSubsystem.toggleClimberLockout());

        climberSubsystem.climberLockoutDisabledTrigger().onTrue(ledSubsystem.solidColorCommand(SparkColor.RED));
        climberSubsystem.climberInState(STATE.CLIMBER_START_HIGHEST).onTrue(ledSubsystem.solidColorCommand(SparkColor.ORANGE));
        climberSubsystem.climberInState(STATE.CLIMBER_END).onTrue(ledSubsystem.solidColorCommand(SparkColor.YELLOW));

        // spotless:on
    }

    public Command getAutonomousCommand() {
        return autonomousRoutineChooser.get();
    }
}
