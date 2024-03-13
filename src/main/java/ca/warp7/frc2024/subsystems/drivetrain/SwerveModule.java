package ca.warp7.frc2024.subsystems.drivetrain;

import static ca.warp7.frc2024.Constants.DRIVETRAIN.WHEEL_RADIUS;

import ca.warp7.frc2024.Constants;
import ca.warp7.frc2024.Constants.DRIVETRAIN;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final SwerveModuleIO moduleIO;
    private final int moduleID;
    private final String moduleName;

    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController driveFeedback;
    private final PIDController steerFeedback;

    private Rotation2d angleSetpoint = null;
    private Double speedSetpoint = null;

    private final SwerveModuleIOInputsAutoLogged moduleInputs = new SwerveModuleIOInputsAutoLogged();
    private Rotation2d finalSteerOffset = null;

    public SwerveModule(SwerveModuleIO moduleIO, int moduleID, String moduleName) {
        this.moduleIO = moduleIO;
        this.moduleID = moduleID;
        this.moduleName = moduleName;

        switch (Constants.CURRENT_MODE) {
            case REAL:
                driveFeedforward = new SimpleMotorFeedforward(0.23466, 0.12025);
                driveFeedback = new PIDController(0.1, 0.0, 0.0);
                steerFeedback = new PIDController(6.5, 0.0, 0);
                break;
            case SIM:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
                driveFeedback = new PIDController(0.1, 0.0, 0.0);
                steerFeedback = new PIDController(10.0, 0.0, 0.0);
                break;
            default:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
                driveFeedback = new PIDController(0.0, 0.0, 0.0);
                steerFeedback = new PIDController(0.0, 0.0, 0.0);
                break;
        }
        // https://www.chiefdelphi.com/t/swerve-modules-flip-180-degrees-periodically-conditionally/393059/11
        steerFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getDistanceMeters() {
        return moduleInputs.drivePositionRad * DRIVETRAIN.WHEEL_DIAMETER / 2;
    }

    public double getSpeedMetersPerSec() {
        return moduleInputs.driveVelocityRadPerSec * DRIVETRAIN.WHEEL_DIAMETER / 2;
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
