package ca.warp7.frc2024.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public Rotation2d climberInternalPosition;
        public Rotation2d climberExternalPosition;
        public double climberVelocityRadPerSec;
        public double climberAppliedVolts = 0.0;
        public double[] climberCurrentAmps = new double[] {};
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setClimberVoltage(double volts) {}
}
