package ca.warp7.frc2024.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    private Double volts = null;
    /** Creates a new intake */
    public IntakeSubsystem(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    public void runVolts(double volts) {
        this.volts = volts;
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Intake/Intake", intakeInputs);

        if (volts != null) {
            intakeIO.setIntakeVoltage(volts);
        }
    }
}
