package ca.warp7.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    public static class ShooterIOInputs {
        // TODO: Make a DCMotorIOInputs maybe
        @AutoLog
        public static class FeederRollerIOInputs {
            public double VelocityRad = 0.0;
            public double VoltageApplied = 0.0;
            public double CurrentDraw = 0.0;
        }

        @AutoLog
        public static class OutRunnerIOInputs {
            public double VelocityRad = 0.0;
            public double VoltageApplied = 0.0;
            public double CurrentDraw = 0.0;
        }

        public FeederRollerIOInputsAutoLogged topFeederRoller = new FeederRollerIOInputsAutoLogged();
        public FeederRollerIOInputsAutoLogged bottomFeederRoller = new FeederRollerIOInputsAutoLogged();

        public OutRunnerIOInputsAutoLogged topRightOutrunner = new OutRunnerIOInputsAutoLogged();
        public OutRunnerIOInputsAutoLogged bottomRightOutrunner = new OutRunnerIOInputsAutoLogged();
        public OutRunnerIOInputsAutoLogged topLeftOutrunner = new OutRunnerIOInputsAutoLogged();
        public OutRunnerIOInputsAutoLogged bottomLeftOutrunner = new OutRunnerIOInputsAutoLogged();
    }

    /** Updates the set of loggable inputs.
     *
     * @param inputs
     */
    public void updateInputs(ShooterIOInputs inputs);

    /** Run the feeder rollers at the specified voltage.
     *
     * @param volts
     */
    public void setFeederRollersVoltage(double volts);

    /** Run the outrunners at the specified voltage.
     *
     * @param volts
     */
    public void setOutrunnersVoltage(double volts);

    /** Code that needs to be run periodically
     *
     */
    public void periodic();
}
