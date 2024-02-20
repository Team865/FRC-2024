package ca.warp7.subsystems.Intake;
import org.littletonrobotics.junction.AutoLog;
public interface intakeIO {
    @AutoLog
    public static class intakeIOInputs {
        public double intakeAppliedVolts = 0.0;
        public double[] intakeCurrentAmps = new double []{}; 
    }
    /*Updates the set of loggable inputs*/
    public void updateInputs(intakeIOInputs inputs);
    /*Set the intake voltage */
    public void setIntakeVoltage(double volts);
}