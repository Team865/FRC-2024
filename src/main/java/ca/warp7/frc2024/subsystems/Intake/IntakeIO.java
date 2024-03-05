package ca.warp7.frc2024.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public Rotation2d intakePosition = new Rotation2d();
        public double intakeVelocityRadPerSec = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrentAmps = 0.0;
        public double intakeTempCelsius = 0.0;

        public boolean intakeSensorTriggered = false;
    }

    /*Updates the set of loggable inputs*/
    public default void updateInputs(IntakeIOInputs inputs) {}

    /*Set the intake voltage */
    public default void setVoltage(double volts) {}
}
