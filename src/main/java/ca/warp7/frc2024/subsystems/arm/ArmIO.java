package ca.warp7.frc2024.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public Rotation2d armInternalIncrementalPosition = new Rotation2d();
        public Rotation2d armExternalIncrementalPosition = new Rotation2d();
        public Rotation2d armExternalAbsolutePosition = new Rotation2d();
        public double armInternalVelocityRadPerSec = 0.0;
        public double armExternalVelocityRadPerSec = 0.0;
        public double[] armAppliedVolts = new double[] {};
        public double[] armCurrentAmps = new double[] {};
        public double[] armTempCelsius = new double[] {};
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setSetpoint(double setpointRads, double feedforward) {}

    public default void configurePID(double kP, double kI, double kD) {}

    public default void stop() {}
}
