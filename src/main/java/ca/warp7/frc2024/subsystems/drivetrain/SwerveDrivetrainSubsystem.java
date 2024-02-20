package ca.warp7.frc2024.subsystems.drivetrain;

import static ca.warp7.frc2024.Constants.DRIVETRAIN.*;
import static ca.warp7.frc2024.Constants.OI.*;

import ca.warp7.frc2024.util.SensitivityGainAdjustment;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrivetrainSubsystem extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final SwerveModule[] swerveModules;

    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(SWERVE_MODULE_TRANSLATIONS);
    private Rotation2d rawGyroRotation = new Rotation2d();

    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(swerveDriveKinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    public SwerveDrivetrainSubsystem(
            GyroIO gyroIO,
            SwerveModuleIO frontRightSwerveModuleIO,
            SwerveModuleIO frontLeftSwerveModuleIO,
            SwerveModuleIO backLeftSwerveModuleIO,
            SwerveModuleIO backRightSwerveModuleIO) {
        this.gyroIO = gyroIO;

        swerveModules = new SwerveModule[] {
            new SwerveModule(frontRightSwerveModuleIO, 0, "Front right"),
            new SwerveModule(frontLeftSwerveModuleIO, 1, "Front left"),
            new SwerveModule(backLeftSwerveModuleIO, 2, "Back left"),
            new SwerveModule(backRightSwerveModuleIO, 3, "Back right")
        };
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drivetrain/Gyro", gyroInputs);

        for (var SwerveModule : swerveModules) {
            SwerveModule.periodic();
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
        if (gyroInputs.connected) {
            rawGyroRotation = gyroInputs.yaw;
        } else {
            Twist2d twist = swerveDriveKinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // Update pose estimator using odometry
        poseEstimator.update(rawGyroRotation, getModulePositions());
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

                    if (robotOrientedSupplier.getAsBoolean()) {
                        swerveDrivetrainSubsystem.setTargetChassisSpeeds(
                                new ChassisSpeeds(xVelocity, yVelocity, omega));
                    } else {
                        swerveDrivetrainSubsystem.setTargetChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                                xVelocity, yVelocity, omega, swerveDrivetrainSubsystem.getRobotRotation()));
                    }
                },
                swerveDrivetrainSubsystem);
    }
}
