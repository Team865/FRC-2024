package ca.warp7.frc2024.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public double feederVelocityRadPerSec = 0.0;
        public double feederAppliedVolts = 0.0;
        public double feederCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs.
     *
     * @param inputs
     */
    public default void updateInputs(FeederIOInputs inputs) {}

    /** Run the feeder rollers at the specified voltage.
     *
     * @param volts
     */
    public default void setFeederVoltage(double volts) {}
}
