package ca.warp7.frc2024.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    public static class ShooterIOInputs {
        @AutoLog
        public static class OutRunnerIOInputs {
            public double VelocityRPM = 0.0;
            public double VoltsApplied = 0.0;
            public double CurrentDraw = 0.0;
        }

        public OutRunnerIOInputsAutoLogged topRight = new OutRunnerIOInputsAutoLogged();
        public OutRunnerIOInputsAutoLogged bottomRight = new OutRunnerIOInputsAutoLogged();
        public OutRunnerIOInputsAutoLogged topLeft = new OutRunnerIOInputsAutoLogged();
        public OutRunnerIOInputsAutoLogged bottomLeft = new OutRunnerIOInputsAutoLogged();
    }

    /** Updates the set of loggable inputs.
     *
     * @param inputs
     */
    public void updateInputs(ShooterIOInputs inputs);

    /** Run the top right outrunner at the specified voltage;
     *
     * @param volts
     */
    public void setTopRightVoltage(double volts);

    /** Run the top left outrunner at the specified voltage;
     *
     * @param volts
     */
    public void setTopLeftVoltage(double volts);

    /** Run the bottom right outrunner at the specified voltage;
     *
     * @param volts
     */
    public void setBottomRightVoltage(double volts);

    /** Run the bottom left outrunner at the specified voltage;
     *
     * @param volts
     */
    public void setBottomLeftVoltage(double volts);
}
