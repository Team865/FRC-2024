package ca.warp7.frc2024.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class intakeIOInputs {
        public double intakeAppliedVolts = 0.0;
        public double[] intakeCurrentAmps = new double[] {};
    }
    /*Updates the set of loggable inputs*/
    public default void updateInputs(intakeIOInputs inputs) {}
    ;
    /*Set the intake voltage */
    public default void setIntakeVoltage(double volts) {}
    ;
}
