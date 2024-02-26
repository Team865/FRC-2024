package ca.warp7.frc2024.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    public static class FeederIOInputs {
        @AutoLog
        public static class FeederRollerIOInputs {
            public double VelocityRPM = 0.0;
            public double VoltsApplied = 0.0;
            public double CurrentDraw = 0.0;
        }

        public FeederRollerIOInputsAutoLogged topRoller = new FeederRollerIOInputsAutoLogged();
        public FeederRollerIOInputsAutoLogged bottomRoller = new FeederRollerIOInputsAutoLogged();
    }

    /** Updates the set of loggable inputs.
     *
     * @param inputs
     */
    public void updateInputs(FeederIOInputs inputs);

    /** Run the top feeder roller at the specified voltage.
     *
     * @param volts
     */
    public void setTopVoltage(double volts);

    /** Run the bottom feeder roller at the specified voltage.
     *
     * @param volts
     */
    public void setBottomVoltage(double volts);
}
