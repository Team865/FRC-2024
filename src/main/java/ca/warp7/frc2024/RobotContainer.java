// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2024;

import ca.warp7.frc2024.Constants.CLIMBER.STATE;
import ca.warp7.frc2024.subsystems.Intake.IntakeIO;
import ca.warp7.frc2024.subsystems.Intake.IntakeIOSim;
import ca.warp7.frc2024.subsystems.Intake.IntakeIOSparkMax;
import ca.warp7.frc2024.subsystems.Intake.IntakeSubsystem;
import ca.warp7.frc2024.subsystems.arm.ArmConstants;
import ca.warp7.frc2024.subsystems.arm.ArmIO;
import ca.warp7.frc2024.subsystems.arm.ArmIOSim;
import ca.warp7.frc2024.subsystems.arm.ArmIOSparkMax;
import ca.warp7.frc2024.subsystems.arm.ArmSubsystemCommands;
import ca.warp7.frc2024.subsystems.climber.ClimberIO;
import ca.warp7.frc2024.subsystems.climber.ClimberIOSim;
import ca.warp7.frc2024.subsystems.climber.ClimberIOSparkMax;
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
import ca.warp7.frc2024.subsystems.shooter.ShooterConstants;
import ca.warp7.frc2024.subsystems.shooter.ShooterModuleIO;
import ca.warp7.frc2024.subsystems.shooter.ShooterModuleIOSim;
import ca.warp7.frc2024.subsystems.shooter.ShooterModuleIOSparkMax550;
import ca.warp7.frc2024.subsystems.shooter.ShooterSubsystemCommands;
import ca.warp7.frc2024.subsystems.vision.VisionIO;
import ca.warp7.frc2024.subsystems.vision.VisionIOLimelight;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private final SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;
    private final ArmSubsystemCommands armSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystemCommands shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private final LEDSubsystem ledSubsystem;

    /* OI Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final LoggedDashboardChooser<Command> autonomousRoutineChooser;

    /* Commands */
    private final Command simpleIntake;
    private final Command simpleFeed;

    private final Command simpleQueue;
    private final Command simpleShoot;
    private final Command simpleAmp;
    private final Command noteFlowForward;
    private final Command noteFlowReverse;
    private final Command stopNoteFlow;

    private final Command intakeFeed;
    private final Command queueShoot;

    private final Command vibrateDriver;
    private final Command vibrateOperator;

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                armSubsystem = new ArmSubsystemCommands(new ArmIOSparkMax(11, 10, 0, 1, 2, new Rotation2d(1.543)));
                intakeSubsystem = new IntakeSubsystem(new IntakeIOSparkMax(31, 4));
                shooterSubsystem = new ShooterSubsystemCommands(
                        new ShooterModuleIOSparkMax550(22, true),
                        new ShooterModuleIOSparkMax550(23, false),
                        new ShooterModuleIOSparkMax550(21, true),
                        new ShooterModuleIOSparkMax550(20, false));
                feederSubsystem = new FeederSubsystem(new FeederIOSparkMax(24, 25, 3));
                climberSubsystem = new ClimberSubsystem(new ClimberIOSparkMax(30));
                ledSubsystem = new LEDSubsystem(0);
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystem(
                        new GyroIONavX() {},
                        new VisionIOLimelight("limelight-front"),
                        new VisionIOLimelight("limelight-rear"),
                        new SwerveModuleIOFalcon500(12, 11, 10, Rotation2d.fromRotations(0.488)),
                        new SwerveModuleIOFalcon500(22, 21, 20, Rotation2d.fromRotations(-0.242)),
                        new SwerveModuleIOFalcon500(32, 31, 30, Rotation2d.fromRotations(0.096)),
                        new SwerveModuleIOFalcon500(42, 41, 40, Rotation2d.fromRotations(0.008)));
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
                armSubsystem = new ArmSubsystemCommands(new ArmIOSim() {});
                intakeSubsystem = new IntakeSubsystem(new IntakeIOSim() {});
                shooterSubsystem = new ShooterSubsystemCommands(
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
                armSubsystem = new ArmSubsystemCommands(new ArmIO() {});
                intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});
                shooterSubsystem = new ShooterSubsystemCommands(
                        new ShooterModuleIO() {},
                        new ShooterModuleIO() {},
                        new ShooterModuleIO() {},
                        new ShooterModuleIO() {});
                feederSubsystem = new FeederSubsystem(new FeederIO() {});
                climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
                ledSubsystem = new LEDSubsystem(0);
        }

        vibrateDriver = Commands.runEnd(
                        () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.25),
                        () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.0))
                .withTimeout(0.25);

        vibrateOperator = Commands.runEnd(
                        () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0.25),
                        () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0.0))
                .withTimeout(0.25);

        simpleIntake = Commands.parallel(
                        intakeSubsystem.runVoltageCommandEnds(10), feederSubsystem.runVoltageCommandEnds(8))
                .until(intakeSubsystem.sensorTrigger());

        simpleFeed = Commands.parallel(
                        shooterSubsystem.runVelocityCommandEnds(-1500, 0, 1, 2, 3),
                        intakeSubsystem.runVoltageCommandEnds(10),
                        feederSubsystem.runVoltageCommandEnds(8))
                .until(feederSubsystem.sensorTrigger());

        // simpleIntake = Commands.parallel(
        //                 ledSubsystem.solidColorCommand(SparkColor.SKY_BLUE),
        //                 intakeSubsystem.runVoltageCommandEnds(10).until(intakeSubsystem.sensorTrigger()),
        //                 feederSubsystem.runVoltageCommandEnds(8).until(intakeSubsystem.sensorTrigger()))
        //         .andThen(
        //                 Commands.parallel(
        //                         shooterSubsystem.runVelocityCommand(-1500, 0, 1, 2, 3),
        //                         ledSubsystem.blinkColorCommand(SparkColor.GREEN, 0.25, 1),
        //                         intakeSubsystem.runVoltageCommandEnds(10).until(feederSubsystem.sensorTrigger()),
        //                         feederSubsystem.runVoltageCommandEnds(8).until(feederSubsystem.sensorTrigger())),
        //                 shooterSubsystem.runVelocityCommand(0, 0, 1, 2, 3));

        simpleQueue = shooterSubsystem
                .runVelocityCommandEnds(-8000, 0, 1, 2, 3)
                .alongWith(feederSubsystem.runVoltageCommandEnds(-3))
                .until(feederSubsystem.sensorTrigger().negate());

        intakeFeed = Commands.sequence(
                ledSubsystem.solidColorCommand(SparkColor.SKY_BLUE),
                simpleIntake,
                simpleFeed,
                Commands.parallel(
                        ledSubsystem.blinkColorCommand(SparkColor.GREEN, 0.25, 1), vibrateDriver, vibrateOperator));

        simpleShoot = Commands.sequence(
                shooterSubsystem.runVelocityCommand(4500, 0, 3),
                shooterSubsystem.runVelocityCommand(9000, 1, 2),
                Commands.waitSeconds(0.75),
                feederSubsystem.runVoltageCommandEnds(12).withTimeout(0.75),
                shooterSubsystem.stopShooterCommand());

        simpleAmp = Commands.sequence(Commands.parallel(
                        shooterSubsystem.runVelocityCommandEnds(-8000, 0, 1, 2, 3),
                        feederSubsystem.runVoltageCommandEnds(-8))
                .withTimeout(1.5));

        queueShoot = Commands.sequence(simpleQueue, simpleShoot);

        noteFlowForward = Commands.parallel(
                intakeSubsystem.runVoltageCommandEnds(8),
                feederSubsystem.runVoltageCommandEnds(8),
                shooterSubsystem.runVelocityCommandEnds(6000, 0, 1, 2, 3));

        noteFlowReverse = Commands.parallel(
                intakeSubsystem.runVoltageCommandEnds(-11),
                feederSubsystem.runVoltageCommandEnds(-8),
                shooterSubsystem.runVelocityCommandEnds(-6000, 0, 1, 2, 3));

        stopNoteFlow = Commands.parallel(
                intakeSubsystem.runVoltageCommandEnds(0),
                feederSubsystem.runVoltageCommandEnds(0),
                shooterSubsystem.stopShooterCommand());

        NamedCommands.registerCommand("ArmStow", armSubsystem.runGoalCommandUntil(ArmConstants.Goal.HANDOFF_INTAKE));
        NamedCommands.registerCommand("ArmSubwoofer", armSubsystem.runGoalCommandUntil(ArmConstants.Goal.SUBWOOFER));
        NamedCommands.registerCommand("ArmPodium", armSubsystem.runGoalCommandUntil(ArmConstants.Goal.PODIUM));
        NamedCommands.registerCommand("Intake", intakeFeed);
        NamedCommands.registerCommand("QueueShoot", queueShoot);

        autonomousRoutineChooser =
                new LoggedDashboardChooser<>("Autonomous Routine Chooser", AutoBuilder.buildAutoChooser());

        if (Constants.TUNING_MODE) {
            // Shooter SysID routines
            autonomousRoutineChooser.addOption(
                    "Shooter quasistatic forward", shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

            autonomousRoutineChooser.addOption(
                    "Shooter quasistatic reverse", shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            autonomousRoutineChooser.addOption(
                    "Shooter dynamic forward", shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));

            autonomousRoutineChooser.addOption(
                    "Shooter dynamic reverse", shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

            autonomousRoutineChooser.addOption("Zero shooter", shooterSubsystem.zeroEncoderCommand());

            // Drivetrain SysID routines
            autonomousRoutineChooser.addOption(
                    "Drivetrain quasistatic forward",
                    swerveDrivetrainSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

            autonomousRoutineChooser.addOption(
                    "Drivetrain quasistatic reverse",
                    swerveDrivetrainSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autonomousRoutineChooser.addOption(
                    "Drivetrain dynamic forward",
                    swerveDrivetrainSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));

            autonomousRoutineChooser.addOption(
                    "Drivetrain dynamic reverse",
                    swerveDrivetrainSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }

        configureDriverBindings();
        configureOperatorBindings();
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

        driver.leftBumper()
                .onTrue(armSubsystem.runGoalCommand(ArmConstants.Goal.HANDOFF_INTAKE))
                .onTrue(armSubsystem.setLockoutCommand(true))
                .onFalse(armSubsystem.setLockoutCommand(false));

        driver.a()
                .onTrue(swerveDrivetrainSubsystem.setPointAtCommand(PointAtLocation.SPEAKER))
                .onFalse(swerveDrivetrainSubsystem.setPointAtCommand(PointAtLocation.NONE));

        driver.b().onTrue(swerveDrivetrainSubsystem.stopWithXCommand());
    }

    private void configureOperatorBindings() {
        // spotless:off
        /* Intaking */
        operator.rightTrigger().and(armSubsystem.atGoalTrigger(ArmConstants.Goal.HANDOFF_INTAKE)).onTrue(simpleIntake);

        /* Arm */
        operator.povDown().onTrue(armSubsystem.runGoalCommand(ArmConstants.Goal.HANDOFF_INTAKE));
        operator.povUp().onTrue(armSubsystem.runGoalCommand(ArmConstants.Goal.PODIUM));
        operator.povRight().onTrue(armSubsystem.runGoalCommand(ArmConstants.Goal.SUBWOOFER));
        operator.povLeft().onTrue(armSubsystem.runGoalCommand(ArmConstants.Goal.AMP));

        operator.y().onTrue(armSubsystem.runGoalCommand(ArmConstants.Goal.BLOCKER));

        /* Scoring */
        operator.a().and(armSubsystem.atGoalTrigger(ArmConstants.Goal.PODIUM)).onTrue(queueShoot);
        operator.a().and(armSubsystem.atGoalTrigger(ArmConstants.Goal.SUBWOOFER)).onTrue(queueShoot);
        operator.a().and(armSubsystem.atGoalTrigger(ArmConstants.Goal.AMP)).onTrue(simpleAmp);

        /* Override Procedures */
        operator.leftBumper().onTrue(noteFlowReverse);
        operator.rightBumper().onTrue(noteFlowForward);
        operator.b().onTrue(stopNoteFlow);

        operator.start().whileTrue(shooterSubsystem.runGoalCommandEnds(ShooterConstants.Goal.TUNING));

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
