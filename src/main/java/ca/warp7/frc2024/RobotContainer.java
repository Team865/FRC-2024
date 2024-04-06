// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2024;

import static ca.warp7.frc2024.Constants.OI.TRIGGER_THRESHOLD;

import ca.warp7.frc2024.FieldConstants.PointOfInterest;
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
import ca.warp7.frc2024.subsystems.drivetrain.DrivetrainConstants.HeadingSnapPoint;
import ca.warp7.frc2024.subsystems.drivetrain.GyroIO;
import ca.warp7.frc2024.subsystems.drivetrain.GyroIONavX;
import ca.warp7.frc2024.subsystems.drivetrain.SwerveDrivetrainSubsystemCommands;
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
    private final SwerveDrivetrainSubsystemCommands swerveDrivetrainSubsystem;
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
    private final Command vibrateDriver;
    private final Command vibrateOperator;

    private final Command simpleIntake;
    private final Command feederIntake;
    private final Command simpleFeed;
    private final Command simpleQueue;
    private final Command simpleRev;
    private final Command simpleRevTrap;
    private final Command simpleRevPassing;

    private final Command simpleShoot;
    private final Command simpleAmp;

    private final Command noteFlowForward;
    private final Command noteFlowReverse;
    private final Command stopNoteFlow;

    private final Command intakeFeed;
    private final Command doubleIntake;
    private final Command queueRev;
    private final Command queueRevPassing;
    private final Command queueRevShoot;
    private final Command queueRevShootTrap;
    private final Command queueRevShootPassing;
    private final Command rollNoteRight;
    private final Command rollNoteLeft;

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
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystemCommands(
                        new GyroIONavX() {},
                        new VisionIOLimelight("limelight-front"),
                        new VisionIOLimelight("limelight-rear"),
                        new SwerveModuleIOFalcon500(12, 11, 10, Rotation2d.fromRotations(0.488)),
                        new SwerveModuleIOFalcon500(22, 21, 20, Rotation2d.fromRotations(-0.242)),
                        new SwerveModuleIOFalcon500(32, 31, 30, Rotation2d.fromRotations(0.096)),
                        new SwerveModuleIOFalcon500(42, 41, 40, Rotation2d.fromRotations(0.008)));
                break;
            case SIM:
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystemCommands(
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
                swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystemCommands(
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
                .withTimeout(0.25)
                .withName("Vibrate Driver Controller");

        vibrateOperator = Commands.runEnd(
                        () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0.25),
                        () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0.0))
                .withTimeout(0.25)
                .withName("Vibrate Operator Controller");

        simpleIntake = Commands.parallel(
                        intakeSubsystem.runVoltageCommandEnds(10), feederSubsystem.runVoltageCommandEnds(8))
                .until(intakeSubsystem.sensorTrigger())
                .withName("Simple Intake");

        feederIntake = Commands.sequence(
                        ledSubsystem.solidColorCommand(SparkColor.VIOLET),
                        Commands.parallel(
                                        shooterSubsystem
                                                .runVelocityCommandEnds(-7000, 0, 1, 2, 3)
                                                .asProxy(),
                                        feederSubsystem
                                                .runVoltageCommandEnds(-6)
                                                .asProxy())
                                .until(feederSubsystem.sensorTrigger()),
                        Commands.parallel(
                                        shooterSubsystem
                                                .runVelocityCommandEnds(-5000, 0, 1, 2, 3)
                                                .asProxy(),
                                        feederSubsystem
                                                .runVoltageCommandEnds(-4)
                                                .asProxy())
                                .until(feederSubsystem.sensorTrigger().negate()),
                        Commands.parallel(
                                        shooterSubsystem
                                                .runVelocityCommandEnds(-1500, 0, 1, 2, 3)
                                                .asProxy(),
                                        feederSubsystem.runVoltageCommandEnds(8).asProxy(),
                                        intakeSubsystem.runVoltageCommandEnds(10))
                                .until(feederSubsystem.sensorTrigger()),
                        Commands.parallel(ledSubsystem
                                .blinkColorCommand(SparkColor.GREEN, 0.25, 1)
                                .asProxy()))
                .withName("Feeder Intake");

        simpleFeed = Commands.parallel(
                        shooterSubsystem.runVelocityCommandEnds(-1500, 0, 1, 2, 3),
                        intakeSubsystem.runVoltageCommandEnds(10),
                        feederSubsystem.runVoltageCommandEnds(8))
                .until(feederSubsystem.sensorTrigger())
                .withName("Simple Feed");

        simpleQueue = Commands.parallel(
                        shooterSubsystem.runVelocityCommandEnds(-8000, 0, 1, 2, 3),
                        feederSubsystem.runVoltageCommandEnds(-3))
                .until(feederSubsystem.sensorTrigger().negate())
                .withName("Simple Queue");

        simpleRev = Commands.sequence(
                        shooterSubsystem.runVelocityCommand(7000, 0, 3),
                        shooterSubsystem.runVelocityCommand(9500, 1, 2),
                        Commands.waitSeconds(0.65))
                .withName("Simple Rev");

        // Commands.sequence(
        //                 shooterSubsystem.runVelocityCommand(7000, 0, 3),
        //                 shooterSubsystem.runVelocityCommand(9500, 1, 2),
        //                 Commands.waitSeconds(0.65))
        //         .withName("Simple Rev");

        simpleRevTrap = Commands.sequence(
                        shooterSubsystem.runGoalCommand(ShooterConstants.Goal.TRAP), Commands.waitSeconds(0.65))
                .withName("Simple Rev Trap");

        simpleRevPassing = Commands.sequence(
                        shooterSubsystem.runGoalCommand(ShooterConstants.Goal.PASSING), Commands.waitSeconds(0.5))
                .withName("Simple Rev Passing");

        simpleShoot = Commands.sequence(
                        feederSubsystem.runVoltageCommandEnds(12).withTimeout(0.15),
                        Commands.waitSeconds(0.25),
                        shooterSubsystem.stopShooterCommand())
                .withName("Simple Shoot");

        simpleAmp = Commands.parallel(
                        shooterSubsystem.runGoalCommandEnds(ShooterConstants.Goal.AMP),
                        feederSubsystem.runVoltageCommandEnds(-8))
                .withTimeout(0.75)
                .withName("Simple Amp");

        doubleIntake = Commands.sequence(
                ledSubsystem.solidColorCommand(SparkColor.SKY_BLUE),
                Commands.race(
                        Commands.parallel(
                                        intakeSubsystem.runVoltageCommandEnds(10),
                                        feederSubsystem.runVoltageCommandEnds(8))
                                .until(intakeSubsystem.sensorTrigger()),
                        shooterSubsystem
                                .runVelocityCommandEnds(-7000, 0, 1, 2, 3)
                                .asProxy()
                                .until(feederSubsystem.sensorTrigger())),
                Commands.either(
                                simpleFeed.asProxy(),
                                Commands.parallel(
                                                shooterSubsystem
                                                        .runVelocityCommandEnds(-5000, 0, 1, 2, 3)
                                                        .asProxy(),
                                                feederSubsystem
                                                        .runVoltageCommandEnds(-6)
                                                        .asProxy())
                                        .until(() ->
                                                intakeSubsystem.sensorTrigger().getAsBoolean()),
                                intakeSubsystem.sensorTrigger())
                        .andThen(Commands.parallel(ledSubsystem.blinkColorCommand(SparkColor.GREEN, 0.25, 1))));

        intakeFeed = Commands.sequence(
                        ledSubsystem.solidColorCommand(SparkColor.SKY_BLUE),
                        simpleIntake.asProxy(),
                        simpleFeed.asProxy(),
                        Commands.parallel(
                                vibrateDriver,
                                vibrateOperator,
                                ledSubsystem.blinkColorCommand(SparkColor.GREEN, 0.25, 1)))
                .withName("Intake Feed");

        queueRev = Commands.sequence(simpleQueue.asProxy(), simpleRev.asProxy()).withName("Queue Rev");
        queueRevPassing = Commands.sequence(simpleQueue.asProxy(), simpleRevPassing.asProxy())
                .withName("Queue Rev Passing");
        queueRevShoot = Commands.sequence(simpleQueue.asProxy(), simpleRev.asProxy(), simpleShoot.asProxy())
                .withName("Queue Rev Shoot");

        queueRevShootTrap = Commands.sequence(simpleQueue.asProxy(), simpleRevTrap.asProxy(), simpleShoot.asProxy())
                .withName("Queue Rev Shoot Trap");

        queueRevShootPassing = Commands.sequence(
                        simpleQueue.asProxy(), simpleRevPassing.asProxy(), simpleShoot.asProxy())
                .withName("Queue Rev Shoot Passing");

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

        rollNoteRight = Commands.parallel(
                shooterSubsystem.runGoalCommandEnds(ShooterConstants.Goal.ROLL_RIGHT),
                intakeSubsystem.runVoltageCommandEnds(10),
                feederSubsystem.runVoltageCommandEnds(8));

        rollNoteLeft = Commands.parallel(
                shooterSubsystem.runGoalCommandEnds(ShooterConstants.Goal.ROLL_LEFT),
                intakeSubsystem.runVoltageCommandEnds(10),
                feederSubsystem.runVoltageCommandEnds(8));

        NamedCommands.registerCommand("ArmStow", armSubsystem.runGoalCommand(ArmConstants.Goal.HANDOFF_INTAKE));
        NamedCommands.registerCommand("ArmSubwoofer", armSubsystem.runGoalCommandUntil(ArmConstants.Goal.SUBWOOFER));
        NamedCommands.registerCommand("ArmPodium", armSubsystem.runGoalCommandUntil(ArmConstants.Goal.PODIUM));
        NamedCommands.registerCommand(
                "ArmInterpolateSpeaker",
                armSubsystem
                        .runInterpolation(
                                () -> swerveDrivetrainSubsystem.getDistanceToPOI(PointOfInterest.SPEAKER_WALL))
                        .until(armSubsystem.atGoalTrigger(ArmConstants.Goal.INTERPOLATION)));
        NamedCommands.registerCommand(
                "ArmInterpolatePodium", armSubsystem.runGoalCommandUntil(ArmConstants.Goal.PODIUM));
        NamedCommands.registerCommand("IntakeFeed", Commands.sequence(simpleIntake.asProxy(), simpleFeed.asProxy()));
        NamedCommands.registerCommand("QueueRev", Commands.sequence(simpleQueue.asProxy(), simpleRev.asProxy()));
        NamedCommands.registerCommand(
                "QueueRevShoot", Commands.sequence(simpleQueue.asProxy(), simpleRev.asProxy(), simpleShoot.asProxy()));
        NamedCommands.registerCommand("Shoot", simpleShoot.asProxy());

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
        swerveDrivetrainSubsystem.setDefaultCommand(swerveDrivetrainSubsystem.teleopDriveCommand(
                () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX(), driver.rightBumper()));

        driver.leftBumper()
                .onTrue(swerveDrivetrainSubsystem.setSpeedMultiplier(0.4))
                .onFalse(swerveDrivetrainSubsystem.setSpeedMultiplier(1.0));

        driver.rightTrigger(TRIGGER_THRESHOLD)
                .and(feederSubsystem.sensorTrigger())
                .toggleOnTrue(armSubsystem.runInterpolation(
                        () -> swerveDrivetrainSubsystem.getDistanceToPOI(PointOfInterest.SPEAKER_WALL)))
                .onTrue(queueRev);

        driver.rightTrigger(TRIGGER_THRESHOLD)
                .onFalse(simpleShoot.asProxy().andThen(shooterSubsystem.stopShooterCommand()));
        driver.rightTrigger(TRIGGER_THRESHOLD)
                .toggleOnFalse(Commands.waitSeconds(0.25)
                        .andThen(armSubsystem.runGoalCommand(ArmConstants.Goal.HANDOFF_INTAKE)));

        // Note rolling
        driver.leftTrigger(TRIGGER_THRESHOLD).and(driver.axisGreaterThan(0, 0)).whileTrue(rollNoteRight);
        driver.leftTrigger(TRIGGER_THRESHOLD).and(driver.axisLessThan(0, 0)).whileTrue(rollNoteLeft);

        driver.rightStick().onTrue(swerveDrivetrainSubsystem.zeroGyroAndPoseCommand());

        driver.leftBumper()
                .onTrue(armSubsystem.runGoalCommand(ArmConstants.Goal.HANDOFF_INTAKE))
                .onTrue(armSubsystem.setLockoutCommand(true))
                .onFalse(armSubsystem.setLockoutCommand(false));

        // Point at speaker
        driver.a()
                .onTrue(swerveDrivetrainSubsystem.setPointAtCommand(PointOfInterest.SPEAKER))
                .onFalse(swerveDrivetrainSubsystem.setPointAtCommand(PointOfInterest.NONE));
        // Snap angle to amp
        driver.x()
                .onTrue(swerveDrivetrainSubsystem.setHeadingSnapCommand(HeadingSnapPoint.AMP))
                .onFalse(swerveDrivetrainSubsystem.setHeadingSnapCommand(HeadingSnapPoint.NONE));
        // Snap angle to feeder
        driver.y()
                .onTrue(swerveDrivetrainSubsystem.setHeadingSnapCommand(HeadingSnapPoint.FEEDER))
                .onFalse(swerveDrivetrainSubsystem.setHeadingSnapCommand(HeadingSnapPoint.NONE));
        // Snap angle to alliance wall
        driver.b()
                .onTrue(swerveDrivetrainSubsystem.setHeadingSnapCommand(HeadingSnapPoint.ALLIANCE_WALL))
                .onFalse(swerveDrivetrainSubsystem.setHeadingSnapCommand(HeadingSnapPoint.NONE));

        // Snap angle to passing shot
        driver.start()
                .onTrue(swerveDrivetrainSubsystem.setHeadingSnapCommand(HeadingSnapPoint.PASSING))
                .onFalse(swerveDrivetrainSubsystem.setHeadingSnapCommand(HeadingSnapPoint.NONE));
    }

    private void configureOperatorBindings() {
        /* Intaking */
        operator.leftTrigger(TRIGGER_THRESHOLD)
                .and(armSubsystem.atGoalTrigger(ArmConstants.Goal.HANDOFF_INTAKE))
                .onTrue(feederIntake);

        operator.rightTrigger(TRIGGER_THRESHOLD)
                .and(armSubsystem.atGoalTrigger(ArmConstants.Goal.HANDOFF_INTAKE))
                .onTrue(intakeFeed);

        /* Arm Override Setpoints */
        operator.povDown().onTrue(armSubsystem.runGoalCommand(ArmConstants.Goal.HANDOFF_INTAKE));
        operator.povUp().onTrue(armSubsystem.runGoalCommand(ArmConstants.Goal.PODIUM));
        operator.povRight().onTrue(armSubsystem.runGoalCommand(ArmConstants.Goal.SUBWOOFER));
        operator.povLeft().onTrue(armSubsystem.runGoalCommand(ArmConstants.Goal.AMP));
        operator.y().onTrue(armSubsystem.runGoalCommand(ArmConstants.Goal.SUBWOOFER_ROBOT));
        operator.x().onTrue(armSubsystem.runGoalCommand(ArmConstants.Goal.TRAP));

        /* Scoring */
        operator.a().and(armSubsystem.atGoalTrigger(ArmConstants.Goal.PODIUM)).onTrue(queueRevShoot);
        operator.a()
                .and(armSubsystem.atGoalTrigger(ArmConstants.Goal.SUBWOOFER))
                .onTrue(queueRevShoot);
        operator.a()
                .and(armSubsystem.atGoalTrigger(ArmConstants.Goal.SUBWOOFER_ROBOT))
                .onTrue(queueRevShoot);
        operator.a().and(armSubsystem.atGoalTrigger(ArmConstants.Goal.AMP)).onTrue(simpleAmp);
        operator.a().and(armSubsystem.atGoalTrigger(ArmConstants.Goal.TRAP)).onTrue(queueRevShootTrap);

        /* Override Procedures */
        operator.leftBumper().whileTrue(noteFlowReverse);
        operator.rightBumper().whileTrue(noteFlowForward);
        operator.b().onTrue(stopNoteFlow);

        operator.start()
                .onTrue(armSubsystem
                        .runGoalCommandUntil(ArmConstants.Goal.PASSING)
                        .andThen(
                                Commands.waitSeconds(1),
                                armSubsystem.runGoalCommandUntil(ArmConstants.Goal.HANDOFF_INTAKE)))
                .onTrue(queueRevShootPassing);
        /* Climbing */
        climberSubsystem.setDefaultCommand(
                ClimberSubsystem.climberCommand(climberSubsystem, () -> operator.getLeftY()));
        operator.leftStick().onTrue(climberSubsystem.toggleClimberLockout());

        climberSubsystem.climberLockoutDisabledTrigger().onTrue(ledSubsystem.solidColorCommand(SparkColor.RED));
    }

    public Command getAutonomousCommand() {
        return autonomousRoutineChooser.get();
    }
}
