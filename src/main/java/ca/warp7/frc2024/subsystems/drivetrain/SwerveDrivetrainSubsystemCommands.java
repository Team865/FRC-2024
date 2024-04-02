package ca.warp7.frc2024.subsystems.drivetrain;

import static ca.warp7.frc2024.Constants.OI.*;
import static ca.warp7.frc2024.subsystems.drivetrain.DrivetrainConstants.*;

import ca.warp7.frc2024.FieldConstants.PointOfInterest;
import ca.warp7.frc2024.subsystems.drivetrain.DrivetrainConstants.HeadingSnapPoint;
import ca.warp7.frc2024.subsystems.vision.VisionIO;
import ca.warp7.frc2024.util.SensitivityGainAdjustment;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.littletonrobotics.junction.Logger;

public class SwerveDrivetrainSubsystemCommands extends SwerveDrivetrainSubsystem {

    public SwerveDrivetrainSubsystemCommands(
            GyroIO gyroIO,
            VisionIO frontVisionIO,
            VisionIO rearVisionIO,
            SwerveModuleIO frontRightSwerveModuleIO,
            SwerveModuleIO frontLeftSwerveModuleIO,
            SwerveModuleIO backLeftSwerveModuleIO,
            SwerveModuleIO backRightSwerveModuleIO) {
        super(
                gyroIO,
                frontVisionIO,
                rearVisionIO,
                frontRightSwerveModuleIO,
                frontLeftSwerveModuleIO,
                backLeftSwerveModuleIO,
                backRightSwerveModuleIO);
    }

    /**
     * @return Command for stop
     */
    public Command stopCommand() {
        return this.runOnce(this::stop);
    }

    /**
     * @return Command for stopWithX
     */
    public Command stopWithXCommand() {
        return this.run(this::stopWithX);
    }

    /**
     * @return Command for zeroGyro
     */
    public Command zeroGyroCommand() {
        return super.runOnce(super::zeroGyro);
    }

    /**
     * @return Command for zeroing pose rotation. Keeps last x & y translation
     */
    public Command zeroGyroAndPoseCommand() {
        return super.runOnce(() -> {
            if (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red) {
                super.setPose(new Pose2d(
                        super.getPose().getTranslation(), new Rotation2d().plus(Rotation2d.fromDegrees(180))));
            } else {
                super.setPose(new Pose2d(super.getPose().getTranslation(), new Rotation2d()));
            }
        });
    }

    public Command setPointAtCommand(PointOfInterest pointAt) {
        return super.runOnce(() -> super.pointAt = pointAt);
    }

    public Command setHeadingSnapCommand(HeadingSnapPoint headingSnapPoint) {
        return super.runOnce(() -> super.headingSnapPoint = headingSnapPoint);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    public Command setSpeedMultiplier(double multiplier) {
        return super.runOnce(() -> super.speedMultiplier = multiplier);
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
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            BooleanSupplier robotOrientedSupplier) {
        return super.run(() -> {
            // Apply deadband, and convert to usable velocities
            double rawxVelocity = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND) * MAX_LINEAR_SPEED;
            double rawyVelocity = MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND) * MAX_LINEAR_SPEED;
            double rawOmega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND) * MAX_ANGULAR_SPEED;

            // Apply sensitivity adjustment
            double xVelocity = SensitivityGainAdjustment.driveGainAdjustment(
                            MathUtil.applyDeadband(xSupplier.getAsDouble() * speedMultiplier, DEADBAND))
                    * MAX_LINEAR_SPEED;
            double yVelocity = SensitivityGainAdjustment.driveGainAdjustment(
                            MathUtil.applyDeadband(ySupplier.getAsDouble() * speedMultiplier, DEADBAND))
                    * MAX_LINEAR_SPEED;
            double omega = SensitivityGainAdjustment.steerGainAdjustment(
                            MathUtil.applyDeadband(omegaSupplier.getAsDouble() * speedMultiplier, DEADBAND))
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

            Translation2d pointAt =
                    super.pointAt == PointOfInterest.NONE ? null : super.pointAt.getAllianceTranslation();

            if (headingSnapPoint != HeadingSnapPoint.NONE && headingSnapPoint != HeadingSnapPoint.HOLD) {
                double rotateOutput = headingSnapFeedback.calculate(
                        super.getRobotRotation().getDegrees(), headingSnapPoint.getAllianceHeading());

                super.setTargetChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                        xVelocity, yVelocity, rotateOutput, super.getRobotRotation()));

            } else if (pointAt != null) {
                /* Aim at code courtesy of FRC 418 */
                double velocityOutput = Math.hypot(xVelocity, yVelocity);

                Pose2d currentPose = super.getPose();
                Rotation2d targetAngle =
                        new Rotation2d(pointAt.getX() - currentPose.getX(), pointAt.getY() - currentPose.getY());

                Vector2D robotVector = new Vector2D(
                        velocityOutput * currentPose.getRotation().getCos(),
                        velocityOutput * currentPose.getRotation().getSin());
                // Vector from robot to target
                Vector2D targetVector = new Vector2D(
                        currentPose.getTranslation().getDistance(pointAt) * targetAngle.getCos(),
                        currentPose.getTranslation().getDistance(pointAt) * targetAngle.getSin());
                // Parallel component of robot's motion to target vector
                Vector2D parallelRobotVector =
                        targetVector.scalarMultiply(robotVector.dotProduct(targetVector) / targetVector.getNormSq());
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
                        ? super.aimAtFeedback.calculate(
                                currentPose.getRotation().getDegrees() + 180, adjustedAngle.getDegrees())
                        : super.aimAtFeedback.calculate(
                                currentPose.getRotation().getDegrees(), adjustedAngle.getDegrees());

                super.setTargetChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                        xVelocity, yVelocity, rotateOutput, super.getRobotRotation()));

            } else {
                if (robotOrientedSupplier.getAsBoolean()) {
                    super.setTargetChassisSpeeds(new ChassisSpeeds(xVelocity, yVelocity, omega));
                } else {
                    super.setTargetChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                            xVelocity, yVelocity, omega, super.getRobotRotation()));
                }
            }
        });
    }
}
