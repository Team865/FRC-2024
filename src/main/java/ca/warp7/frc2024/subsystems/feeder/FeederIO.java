package ca.warp7.frc2024.subsystems.feeder;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public Rotation2d feederPosition = new Rotation2d();
        public double feederVelocityRadPerSec = 0.0;
        public double[] feederAppliedVolts = new double[] {};
        public double[] feederCurrentAmps = new double[] {};
        public double[] feederTempCelsius = new double[] {};

        public boolean feederSensorTriggered = false;
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
    public default void setVoltage(double volts) {}

    public default void stop() {}
}
