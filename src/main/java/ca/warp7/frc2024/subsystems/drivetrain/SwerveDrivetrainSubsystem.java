package ca.warp7.frc2024.subsystems.drivetrain;

import static ca.warp7.frc2024.Constants.DRIVETRAIN.*;
import static ca.warp7.frc2024.Constants.OI.*;
import static ca.warp7.frc2024.subsystems.drivetrain.DrivetrainConstants.*;
import static edu.wpi.first.units.Units.Volts;

import ca.warp7.frc2024.FieldConstants.PointOfInterest;
import ca.warp7.frc2024.subsystems.vision.VisionIO;
import ca.warp7.frc2024.subsystems.vision.VisionIOInputsAutoLogged;
import ca.warp7.frc2024.util.LoggedTunableNumber;
import ca.warp7.frc2024.util.SensitivityGainAdjustment;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrivetrainSubsystem extends SubsystemBase {
    /* AdvantageKit */
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final VisionIO frontVisionIO;
    private final VisionIOInputsAutoLogged frontVisionInputs = new VisionIOInputsAutoLogged();

    private final VisionIO rearVisionIO;
    private final VisionIOInputsAutoLogged rearVisionInputs = new VisionIOInputsAutoLogged();

    /* Drivetrain */
    private final SwerveModule[] swerveModules = new SwerveModule[4];
    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(SWERVE_MODULE_TRANSLATIONS);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            swerveDriveKinematics,
            rawGyroRotation,
            lastModulePositions,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, 0.005),
            VecBuilder.fill(2, 2, 999999999));

    /* Controllers */
    private final PIDController aimAtFeedback;

    /* Gains */
    private final LoggedTunableNumber aimAtkP = new LoggedTunableNumber("Drivetrain/Gains/AimAt/kP", 0.15);
    private final LoggedTunableNumber aimAtkI = new LoggedTunableNumber("Drivetrain/Gains/AimAt/kI", 0);
    private final LoggedTunableNumber aimAtkD = new LoggedTunableNumber("Drivetrain/Gains/AimAt/kD", 0);

    private final LoggedTunableNumber fudge = new LoggedTunableNumber("Drivetrain/Fudge", 0.1);

    private final LoggedTunableNumber drivekS = new LoggedTunableNumber("Drivetrain/Gains/Drive/kS", DRIVE_GAINS.kS());
    private final LoggedTunableNumber drivekV = new LoggedTunableNumber("Drivetrain/Gains/Drive/kV", DRIVE_GAINS.kV());

    private final LoggedTunableNumber drivekP = new LoggedTunableNumber("Drivetrain/Gains/Drive/kP", DRIVE_GAINS.kP());
    private final LoggedTunableNumber drivekI = new LoggedTunableNumber("Drivetrain/Gains/Drive/kI", DRIVE_GAINS.kI());
    private final LoggedTunableNumber drivekD = new LoggedTunableNumber("Drivetrain/Gains/Drive/kD", DRIVE_GAINS.kD());

    private final LoggedTunableNumber steerkP = new LoggedTunableNumber("Drivetrain/Gains/Steer/kP", STEER_GAINS.kP());
    private final LoggedTunableNumber steerkI = new LoggedTunableNumber("Drivetrain/Gains/Steer/kI", STEER_GAINS.kI());
    private final LoggedTunableNumber steerkD = new LoggedTunableNumber("Drivetrain/Gains/Steer/kD", STEER_GAINS.kD());

    private final SysIdRoutine sysId;

    private PointOfInterest pointAt = PointOfInterest.NONE;
    private boolean reversePointing = true;

    public SwerveDrivetrainSubsystem(
            GyroIO gyroIO,
            VisionIO frontVisionIO,
            VisionIO rearVisionIO,
            SwerveModuleIO frontRightSwerveModuleIO,
            SwerveModuleIO frontLeftSwerveModuleIO,
            SwerveModuleIO backLeftSwerveModuleIO,
            SwerveModuleIO backRightSwerveModuleIO) {
        this.gyroIO = gyroIO;
        this.frontVisionIO = frontVisionIO;
        this.rearVisionIO = rearVisionIO;

        swerveModules[0] = new SwerveModule(frontRightSwerveModuleIO, "FrontRight");
        swerveModules[1] = new SwerveModule(frontLeftSwerveModuleIO, "FrontLeft");
        swerveModules[2] = new SwerveModule(backLeftSwerveModuleIO, "BackLeft");
        swerveModules[3] = new SwerveModule(backRightSwerveModuleIO, "BackRight");

        // Create and configure feedback controller
        aimAtFeedback = new PIDController(aimAtkP.get(), aimAtkI.get(), aimAtkD.get());
        aimAtFeedback.enableContinuousInput(-180, 180);

        // Reset robot pose
        setPose(new Pose2d(0, 0, new Rotation2d()));

        // Set initial gains
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDriveFeedforwardGains(drivekS.get(), drivekV.get());
            swerveModules[i].setDriveFeedbackGains(drivekP.get(), drivekI.get(), drivekD.get());
            swerveModules[i].setSteerFeedbackGains(steerkP.get(), steerkI.get(), steerkD.get());
        }

        // Configure PathPlanner
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                () -> swerveDriveKinematics.toChassisSpeeds(getModuleStates()),
                this::setTargetChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(2.5),
                        new PIDConstants(2.5),
                        MAX_LINEAR_SPEED,
                        DRIVE_BASE_RADIUS,
                        new ReplanningConfig(true, true, 1, 0.05)),
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red,
                this);

        // Log PathPlanner
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Drivetrain/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Drivetrain/Odometry/TrajectoryTarget", targetPose);
        });

        // Configure SysID Routine
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Drivetrain/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            for (int i = 0; i < 4; i++) {
                                swerveModules[i].runCharacterization(voltage.in(Volts));
                            }
                        },
                        null,
                        this));
    }

    @Override
    public void periodic() {
        // Update and process inputs
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drivetrain/Gyro", gyroInputs);

        frontVisionIO.updateInputs(frontVisionInputs);
        Logger.processInputs("Drivetrain/Vision/Front", frontVisionInputs);

        rearVisionIO.updateInputs(rearVisionInputs);
        Logger.processInputs("Drivetrain/Vision/Rear", rearVisionInputs);

        // Run swerve module periodic routines
        for (var module : swerveModules) {
            module.periodic();
        }

        // Calculate last module position
        SwerveModulePosition[] modulePositions = getModulePositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            moduleDeltas[i] = new SwerveModulePosition(
                    modulePositions[i].distanceMeters - lastModulePositions[i].distanceMeters,
                    modulePositions[i].angle);
            lastModulePositions[i] = modulePositions[i];
        }

        // Use real inputs if gyro exists, otherwise generate from kinematics
        if (gyroInputs.gyroConnected) {
            rawGyroRotation = gyroInputs.gyroYaw;
        } else {
            Twist2d twist = swerveDriveKinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // Update pose estimator using odometry
        poseEstimator.update(rawGyroRotation, modulePositions);

        // if (rearVisionInputs.tagCount >= 2 && rearVisionInputs.avgTagDist <= 4.5) {
        //     poseEstimator.addVisionMeasurement(
        //             rearVisionInputs.blueOriginRobotPose, rearVisionInputs.timestamp, VecBuilder.fill(1, 1,
        // 999999999));
        // }

        updatePoseEstimateWithVision();

        Logger.recordOutput("Drivetrain/DistanceToSpeakerWall", getDistanceToPOI(PointOfInterest.SPEAKER_WALL));
        Logger.recordOutput("Drivetrain/DistanceToAmp", getDistanceToPOI(PointOfInterest.AMP));

        // Update if PID gains have changed
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    aimAtFeedback.setP(aimAtkP.get());
                    aimAtFeedback.setI(aimAtkI.get());
                    aimAtFeedback.setD(aimAtkD.get());
                },
                aimAtkP,
                aimAtkI,
                aimAtkD);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    for (int i = 0; i < 4; i++) {
                        swerveModules[i].setDriveFeedforwardGains(drivekS.get(), drivekV.get());
                    }
                },
                drivekS,
                drivekV);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    for (int i = 0; i < 4; i++) {
                        swerveModules[i].setDriveFeedbackGains(drivekP.get(), drivekI.get(), drivekD.get());
                    }
                },
                drivekP,
                drivekI,
                drivekD);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    for (int i = 0; i < 4; i++) {
                        swerveModules[i].setSteerFeedbackGains(steerkP.get(), steerkI.get(), steerkD.get());
                    }
                },
                steerkP,
                steerkI,
                steerkD);
    }

    public void updatePoseEstimateWithVision() {
        if (rearVisionInputs.tagCount >= 1) {
            double xyStds;
            double rotStds = 999999999;

            double poseDifference = poseEstimator
                    .getEstimatedPosition()
                    .getTranslation()
                    .getDistance(rearVisionInputs.blueOriginRobotPose.getTranslation());

            if (rearVisionInputs.tagCount >= 2 && rearVisionInputs.avgTagDist <= 3.65) {
                xyStds = 1.5;
            } else if (rearVisionInputs.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 2.5;
            } else {
                return;
            }

            poseEstimator.addVisionMeasurement(
                    rearVisionInputs.blueOriginRobotPose,
                    rearVisionInputs.timestamp,
                    VecBuilder.fill(xyStds, xyStds, rotStds));
        }
    }
    /**
     * Drive at desired velocities
     *
     * @param speeds Velocities in ChassisSpeeds object
     */
    public void setTargetChassisSpeeds(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_LINEAR_SPEED);

        // Set individual modules target state
        SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            optimizedStates[i] = swerveModules[i].setTargetState(swerveModuleStates[i]);
        }

        // Log states
        Logger.recordOutput("Drivetrain/Swerve/TargetStates", swerveModuleStates);
        Logger.recordOutput("Drivetrain/Swerve/TargetStatesOptimized", optimizedStates);
    }

    /**
     * @return Current pose from pose estimator
     */
    @AutoLogOutput(key = "Drivetrain/Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Drivetrain/DistanceToPointOfInterest")
    public double getDistanceToPOI(PointOfInterest POI) {
        return getPose().getTranslation().getDistance(POI.getAllianceTranslation());
    }

    /**
     * @return Current rotation from pose estimator
     */
    public Rotation2d getRobotRotation() {
        return getPose().getRotation();
    }

    /**
     * Resets pose estimator to provided pose
     * @param pose
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * @return An array of module positions
     */
    @AutoLogOutput(key = "Drivetrain/Swerve/Positions")
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = swerveModules[i].getPosition();
        }
        return positions;
    }

    /**
     * @return An array of module states
     */
    @AutoLogOutput(key = "Drivetrain/Swerve/States")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    /**
     * Stop the drivetrain by setting empty chassis speed
     */
    private void stop() {
        setTargetChassisSpeeds(new ChassisSpeeds());
    }

    /**
     * @return Command for stop
     */
    public Command stopCommand() {
        return this.runOnce(this::stop);
    }

    /**
     * Puts swerve wheels in an x pattern to prevent movement
     */
    private void stopWithX() {
        swerveModules[0].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        swerveModules[1].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
        swerveModules[2].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        swerveModules[3].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
    }

    /**
     * @return Command for stopWithX
     */
    public Command stopWithXCommand() {
        return this.run(this::stopWithX);
    }

    /**
     * Zeroes encoder on a hardware level
     */
    private void zeroGyro() {
        gyroIO.zeroYaw();
    }

    /**
     * @return Command for zeroGyro
     */
    public Command zeroGyroCommand() {
        return this.runOnce(this::zeroGyro);
    }

    public Command zeroGyroAndPoseCommand() {
        return this.runOnce(() -> {
            if (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red) {
                this.setPose(new Pose2d(
                        this.getPose().getTranslation(), new Rotation2d().plus(Rotation2d.fromDegrees(180))));
            } else {
                this.setPose(new Pose2d(this.getPose().getTranslation(), new Rotation2d()));
            }
        });
    }

    public Command setPointAtCommand(PointOfInterest pointAt) {
        return this.runOnce(() -> this.pointAt = pointAt);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    /**
     * Drive command for teleoperated control. Defaults to field oriented
     *
     * @param swerveDrivetrainSubsystem Swerve drivetrain subsystem
     * @param xSupplier Supplier for the x component of velocity
     * @param ySupplier Supplier for the y component of velocity
     * @param angleSupplier Supplier for the omega (or angle) velocity
     * @param robotOrientedSupplier Supplier for whether to use robot oriented drive over field oriented
     */
    public Command teleopDriveCommand(
            SwerveDrivetrainSubsystem swerveDrivetrainSubsystem,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            BooleanSupplier robotOrientedSupplier) {
        return Commands.run(
                () -> {
                    // Apply deadband, and convert to usable velocities
                    double rawxVelocity = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND) * MAX_LINEAR_SPEED;
                    double rawyVelocity = MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND) * MAX_LINEAR_SPEED;
                    double rawOmega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND) * MAX_ANGULAR_SPEED;

                    // Apply sensitivity adjustment
                    double xVelocity = SensitivityGainAdjustment.driveGainAdjustment(
                                    MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND))
                            * MAX_LINEAR_SPEED;
                    double yVelocity = SensitivityGainAdjustment.driveGainAdjustment(
                                    MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND))
                            * MAX_LINEAR_SPEED;
                    double omega = SensitivityGainAdjustment.steerGainAdjustment(
                                    MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND))
                            * MAX_ANGULAR_SPEED;

                    if (DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red) {
                        xVelocity *= -1.0;
                        yVelocity *= -1.0;
                    }

                    // Log raw inputs
                    Logger.recordOutput("OI/Raw x Velocity", rawxVelocity);
                    Logger.recordOutput("OI/Raw y Velocity", rawyVelocity);
                    Logger.recordOutput("OI/Raw Omega", rawOmega);
                    Logger.recordOutput("OI/Robot Oriented", robotOrientedSupplier.getAsBoolean());

                    // Log adjusted velocities
                    Logger.recordOutput("OI/x Velocity", xVelocity);
                    Logger.recordOutput("OI/y Velocity", yVelocity);
                    Logger.recordOutput("OI/Omega", omega);

                    Translation2d pointAt = swerveDrivetrainSubsystem.pointAt == PointOfInterest.NONE
                            ? null
                            : swerveDrivetrainSubsystem.pointAt.getAllianceTranslation();

                    /* Aim at code courtesy of FRC 418 */
                    if (pointAt != null) {
                        double velocityOutput = Math.hypot(xVelocity, yVelocity);
                        double moveDirection = Math.atan2(yVelocity, xVelocity);

                        Pose2d currentPose = swerveDrivetrainSubsystem.getPose();
                        Rotation2d targetAngle = new Rotation2d(
                                pointAt.getX() - currentPose.getX(), pointAt.getY() - currentPose.getY());

                        Vector2D robotVector = new Vector2D(
                                velocityOutput * currentPose.getRotation().getCos(),
                                velocityOutput * currentPose.getRotation().getSin());
                        // Aim point
                        Translation2d aimPoint =
                                pointAt.minus(new Translation2d(robotVector.getX(), robotVector.getY()));
                        // Vector from robot to target
                        Vector2D targetVector = new Vector2D(
                                currentPose.getTranslation().getDistance(pointAt) * targetAngle.getCos(),
                                currentPose.getTranslation().getDistance(pointAt) * targetAngle.getSin());
                        // Parallel component of robot's motion to target vector
                        Vector2D parallelRobotVector = targetVector.scalarMultiply(
                                robotVector.dotProduct(targetVector) / targetVector.getNormSq());
                        // Perpendicular component of robot's motion to target vector
                        Vector2D perpendicularRobotVector =
                                robotVector.subtract(parallelRobotVector).scalarMultiply(fudge.get());
                        // Adjust aim point using calculated vector
                        Translation2d adjustedPoint = pointAt.minus(
                                new Translation2d(perpendicularRobotVector.getX(), perpendicularRobotVector.getY()));
                        // Calculate new angle using adjusted point
                        Rotation2d adjustedAngle = new Rotation2d(
                                adjustedPoint.getX() - currentPose.getX(), adjustedPoint.getY() - currentPose.getY());

                        double rotateOutput = reversePointing
                                ? swerveDrivetrainSubsystem.aimAtFeedback.calculate(
                                        currentPose.getRotation().getDegrees() + 180, adjustedAngle.getDegrees())
                                : swerveDrivetrainSubsystem.aimAtFeedback.calculate(
                                        currentPose.getRotation().getDegrees(), adjustedAngle.getDegrees());

                        swerveDrivetrainSubsystem.setTargetChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                                velocityOutput * Math.cos(moveDirection),
                                velocityOutput * Math.sin(moveDirection),
                                rotateOutput,
                                swerveDrivetrainSubsystem.getRobotRotation()));

                    } else {
                        if (robotOrientedSupplier.getAsBoolean()) {
                            swerveDrivetrainSubsystem.setTargetChassisSpeeds(
                                    new ChassisSpeeds(xVelocity, yVelocity, omega));
                        } else {
                            swerveDrivetrainSubsystem.setTargetChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                                    xVelocity, yVelocity, omega, swerveDrivetrainSubsystem.getRobotRotation()));
                        }
                    }
                },
                this);
    }
}
