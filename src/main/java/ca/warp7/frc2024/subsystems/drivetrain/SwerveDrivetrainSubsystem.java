package ca.warp7.frc2024.subsystems.drivetrain;

import static ca.warp7.frc2024.Constants.DRIVETRAIN.*;
import static ca.warp7.frc2024.Constants.OI.*;

import ca.warp7.frc2024.subsystems.vision.VisionIO;
import ca.warp7.frc2024.subsystems.vision.VisionIOInputsAutoLogged;
import ca.warp7.frc2024.util.SensitivityGainAdjustment;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrivetrainSubsystem extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final VisionIO frontVisionIO;
    private final VisionIOInputsAutoLogged frontVisionInputs = new VisionIOInputsAutoLogged();

    private final VisionIO rearVisionIO;
    private final VisionIOInputsAutoLogged rearVisionInputs = new VisionIOInputsAutoLogged();

    private final SwerveModule[] swerveModules;

    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(SWERVE_MODULE_TRANSLATIONS);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final PIDController aimController;

    @Getter
    @Setter
    private Translation2d pointAt;

    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(swerveDriveKinematics, rawGyroRotation, lastModulePositions, new Pose2d());

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

        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));

        swerveModules = new SwerveModule[] {
            new SwerveModule(frontRightSwerveModuleIO, 0, "FrontRight"),
            new SwerveModule(frontLeftSwerveModuleIO, 1, "FrontLeft"),
            new SwerveModule(backLeftSwerveModuleIO, 2, "BackLeft"),
            new SwerveModule(backRightSwerveModuleIO, 3, "BackRight")
        };

        aimController = new PIDController(0.04, 0, 0);
        aimController.enableContinuousInput(-180, 180);

        setPose(new Pose2d(0, 0, new Rotation2d()));

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                () -> swerveDriveKinematics.toChassisSpeeds(getModuleStates()),
                this::setTargetChassisSpeeds,
                new HolonomicPathFollowerConfig(MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red,
                this);

        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectoryTarget", targetPose);
        });
    }

    @Override
    public void periodic() {
        /* Update and process inputs */
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Gyro", gyroInputs);

        frontVisionIO.updateInputs(frontVisionInputs);
        Logger.processInputs("Vision/Front", frontVisionInputs);

        rearVisionIO.updateInputs(rearVisionInputs);
        Logger.processInputs("Vision/Front", rearVisionInputs);

        for (var module : swerveModules) {
            module.periodic();
        }

        if (DriverStation.isDisabled()) {}

        // Calculate last module position
        SwerveModulePosition[] modulePositions = getModulePositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int i = 0; i <= 3; i++) {
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
        poseEstimator.update(rawGyroRotation, getModulePositions());

        // if (visionInputs[1].tagCount > 1) {
        //     poseEstimator.addVisionMeasurement(visionInputs[1].blueOriginRobotPose, visionInputs[1].timestamp);
        // }

        // Update pose estimator using vision
        // for (VisionIOInputsAutoLogged io : visionInputs) {
        //     // Only update if more than 1 tag is seen per limelight and we're close
        //     if (io.tagCount > 1 && io.avgTagDist < 5.0) {
        //         poseEstimator.addVisionMeasurement(io.blueOriginRobotPose, io.timestamp);
        //     }
        // }
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
        for (int i = 0; i <= 3; i++) {
            optimizedStates[i] = swerveModules[i].setTargetState(swerveModuleStates[i]);
        }

        // Log states
        Logger.recordOutput("SwerveStates/States", swerveModuleStates);
        Logger.recordOutput("SwerveStates/StatesOptimized", optimizedStates);
    }

    /**
     * @return Current pose from pose estimator
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * @return Current rotation from pose estimator
     */
    public Rotation2d getRobotRotation() {
        return getPose().getRotation();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * @return An array of module positions
     */
    @AutoLogOutput(key = "Swerve/Positions")
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
    @AutoLogOutput(key = "Swerve/States")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    public void stop() {
        setTargetChassisSpeeds(new ChassisSpeeds());
    }

    public void stopWithX() {
        swerveModules[0].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        swerveModules[1].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
        swerveModules[2].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        swerveModules[3].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
    }

    private void zeroGyro() {
        gyroIO.zeroYaw();
    }

    public Command zeroGyroCommand() {
        return this.runOnce(this::zeroGyro);
    }

    // public static Command aimAtPointCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier
    // omegaSupplier, Supplier<Translation2d> pointSupplier) {
    //     return runEnd(() -> )
    // }

    /**
     * Drive command for teleoperated control. Defaults to field oriented
     *
     * @param swerveDrivetrainSubsystem Swerve drivetrain subsystem
     * @param xSupplier Supplier for the x component of velocity
     * @param ySupplier Supplier for the y component of velocity
     * @param angleSupplier Supplier for the omega (or angle) velocity
     * @param robotOrientedSupplier Supplier for whether to use robot oriented drive over field oriented
     */
    public static Command teleopDriveCommand(
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

                    // Log raw inputs
                    Logger.recordOutput("OI/Raw x Velocity", rawxVelocity);
                    Logger.recordOutput("OI/Raw y Velocity", rawyVelocity);
                    Logger.recordOutput("OI/Raw Omega", rawOmega);
                    Logger.recordOutput("OI/Robot Oriented", robotOrientedSupplier.getAsBoolean());

                    // Log adjusted velocities
                    Logger.recordOutput("OI/x Velocity", xVelocity);
                    Logger.recordOutput("OI/y Velocity", yVelocity);
                    Logger.recordOutput("OI/Omega", omega);

                    Translation2d pointAt = swerveDrivetrainSubsystem.getPointAt();

                    // Aim at code courtesy of FRC 418
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
                        Vector2D perpendicularRobotVector = robotVector.subtract(parallelRobotVector);
                        // Adjust aim point using calculated vector
                        Translation2d adjustedPoint = pointAt.minus(
                                new Translation2d(perpendicularRobotVector.getX(), perpendicularRobotVector.getY()));
                        // Calculate new angle using adjusted point
                        Rotation2d adjustedAngle = new Rotation2d(
                                adjustedPoint.getX() - currentPose.getX(), adjustedPoint.getY() - currentPose.getY());

                        double rotateOutput = swerveDrivetrainSubsystem.aimController.calculate(
                                currentPose.getRotation().getDegrees() + 180, adjustedAngle.getDegrees());
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
                swerveDrivetrainSubsystem);
    }
}
