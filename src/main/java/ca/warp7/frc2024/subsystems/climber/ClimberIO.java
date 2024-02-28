package ca.warp7.frc2024.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double climberInternalPositionRad;
        public Rotation2d climberExternalPositionRad;
        public double climberVelocityRadPerSec;
        public double climberAppliedVolts = 0.0;
        public double climberCurrentAmps = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setClimberVoltage(double volts) {}
}
