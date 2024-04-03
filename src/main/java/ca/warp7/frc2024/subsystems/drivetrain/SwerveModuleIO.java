package ca.warp7.frc2024.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTemperatureCelsius = 0.0;

        public Rotation2d steerAbsolutePosition = new Rotation2d();
        public Rotation2d steerPosition = new Rotation2d();
        public double steerVelocityRadPerSec = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerCurrentAmps = 0.0;
        public double steerTemperatureCelsius = 0.0;
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    public default void setDriveVoltage(double volts) {}

    public default void setSteerVoltage(double volts) {}
}
