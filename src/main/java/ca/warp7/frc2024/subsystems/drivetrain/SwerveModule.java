package ca.warp7.frc2024.subsystems.drivetrain;

import static ca.warp7.frc2024.subsystems.drivetrain.DrivetrainConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final SwerveModuleIO moduleIO;
    private final String moduleName;

    private SimpleMotorFeedforward driveFeedforward;
    private final PIDController driveFeedback;
    private final PIDController steerFeedback;

    private Rotation2d angleSetpoint = null;
    private Double speedSetpoint = null;

    private final SwerveModuleIOInputsAutoLogged moduleInputs = new SwerveModuleIOInputsAutoLogged();
    private Rotation2d finalSteerOffset = null;

    public SwerveModule(SwerveModuleIO moduleIO, String moduleName) {
        this.moduleIO = moduleIO;
        this.moduleName = moduleName;

        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
        driveFeedback = new PIDController(0.0, 0.0, 0.0);
        steerFeedback = new PIDController(0.0, 0.0, 0.0);

        steerFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getDistanceMeters() {
        return moduleInputs.drivePositionRad * WHEEL_DIAMETER / 2;
    }

    public double getSpeedMetersPerSec() {
        return moduleInputs.driveVelocityRadPerSec * WHEEL_DIAMETER / 2;
    }

    public Rotation2d getAngle() {
        return finalSteerOffset == null ? new Rotation2d() : moduleInputs.steerPosition.plus(finalSteerOffset);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistanceMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeedMetersPerSec(), getAngle());
    }

    public SwerveModuleState setTargetState(SwerveModuleState state) {
        var optimizedState = SwerveModuleState.optimize(state, getAngle());

        angleSetpoint = optimizedState.angle;
        speedSetpoint = optimizedState.speedMetersPerSecond;

        // Return state for logging
        return optimizedState;
    }

    public void setDriveFeedforwardGains(double kS, double kV) {
        driveFeedforward = new SimpleMotorFeedforward(kS, kV);
    }

    public void setDriveFeedbackGains(double kP, double kI, double kD) {
        driveFeedback.setP(kP);
        driveFeedback.setI(kI);
        driveFeedback.setD(kD);
    }

    public void setSteerFeedbackGains(double kP, double kI, double kD) {
        steerFeedback.setP(kP);
        steerFeedback.setI(kI);
        steerFeedback.setD(kD);
    }

    public void runCharacterization(double volts) {
        angleSetpoint = new Rotation2d();

        moduleIO.setDriveVoltage(volts);
        speedSetpoint = null;
    }

    public void periodic() {
        moduleIO.updateInputs(moduleInputs);
        Logger.processInputs("Drivetrain/" + moduleName + "Module", moduleInputs);

        if (finalSteerOffset == null && moduleInputs.steerAbsolutePosition.getRadians() != 0.0) {
            finalSteerOffset = moduleInputs.steerAbsolutePosition.minus(moduleInputs.steerPosition);
        }

        if (angleSetpoint != null) {
            double steerVoltage = steerFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians());
            moduleIO.setSteerVoltage(steerVoltage);

            if (speedSetpoint != null) {
                double adjustedSpeedSetpoint = speedSetpoint * Math.cos(steerFeedback.getPositionError());

                double velocityRadPerSec = adjustedSpeedSetpoint / WHEEL_RADIUS;

                double driveVoltage = driveFeedforward.calculate(velocityRadPerSec)
                        + driveFeedback.calculate(moduleInputs.driveVelocityRadPerSec, velocityRadPerSec);

                moduleIO.setDriveVoltage(driveVoltage);
            }
        }
    }
}
