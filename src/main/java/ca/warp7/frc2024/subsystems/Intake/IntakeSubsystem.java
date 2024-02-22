package ca.warp7.frc2024.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
    /** Creates a new intake */
    public IntakeSubsystem(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    public Command setIntakeVoltage(double volts) {
        return runOnce(() -> intakeIO.setIntakeVoltage(volts));
}

    @Override
    public void periodic() {
        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Intake/inputs", intakeInputs);
    }
}
