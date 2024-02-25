package ca.warp7.frc2024.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double shooterVelocityRadPerSec = 0.0;
        public double shooterAppliedVolts = 0.0;
        public double shooterCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs.
     *
     * @param inputs
     */
    public void updateInputs(ShooterIOInputs inputs);

    /** Run the top right shooter at the specified voltage;
     *
     * @param volts
     */
    public void setTopRightVoltage(double volts);

    /** Run the top left shooter at the specified voltage;
     *
     * @param volts
     */
    public void setTopLeftVoltage(double volts);

    /** Run the bottom right shooter at the specified voltage;
     *
     * @param volts
     */
    public void setBottomRightVoltage(double volts);

    /** Run the bottom left shooter at the specified voltage;
     *
     * @param volts
     */
    public void setBottomLeftVoltage(double volts);
}
