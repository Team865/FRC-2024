package ca.warp7.subsystems.Intake;
import org.littletonrobotics.junction.AutoLog;

public interface intakeIO {
    @AutoLog
    public static class intakeIOInputs {
    }
    public void setIntakeSpeed(double volts);
}