package ca.warp7.frc2024.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

import ca.warp7.frc2024.subsystems.feeder.FeederRollerIOInputsAutoLogged;

public interface FeederIO {
    public static class FeederIOInputs {
        @AutoLog
        public static class FeederRollerIOInputs {
            public double VelocityRad = 0.0;
            public double VoltageApplied = 0.0;
            public double CurrentDraw = 0.0;
        }

        public FeederRollerIOInputsAutoLogged topFeederRoller = new FeederRollerIOInputsAutoLogged();
        public FeederRollerIOInputsAutoLogged bottomFeederRoller = new FeederRollerIOInputsAutoLogged();
    }

    /** Updates the set of loggable inputs.
     *
     * @param inputs
     */
    public void updateInputs(FeederIOInputs inputs);

    /** Run the feeder rollers at the specified voltage.
     *
     * @param volts
     */
    public void setFeederRollersVoltage(double volts);

    /** Code that needs to be run periodically
     *
     */
    public void periodic();
}
