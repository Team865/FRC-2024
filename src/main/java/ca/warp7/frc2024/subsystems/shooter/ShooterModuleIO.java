package ca.warp7.frc2024.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterModuleIO {
    @AutoLog
    public static class ShooterModuleIOInputs {
        public double shooterVelocityRadPerSec = 0.0;
        public double shooterAppliedVolts = 0.0;
        public double shooterCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs.
     *
     * @param inputs
     */
    public default void updateInputs(ShooterModuleIOInputs inputs) {}

    /** Run the shooter module at the specified voltage;
     *
     * @param volts
     */
    public default void setShooterVoltage(double volts) {}
}
