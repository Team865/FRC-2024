package ca.warp7.frc2024.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public Rotation2d climberInternalPosition;
        public Rotation2d climberExternalPosition;
        public double climberInternalVelocityRadPerSec;
        public double climberExternalVelocityRadPerSec;
        public double climberAppliedVolts = 0.0;
        public double climberCurrentAmps = 0.0;
        public double climberTempCelsius = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setVoltage(double volts) {}
}
