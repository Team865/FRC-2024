package ca.warp7.frc2024.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO intakeIO;
    /** Creates a new intake */
    public IntakeSubsystem(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    public Command setIntakeVoltage(double volts) {
        return runOnce(() -> intakeIO.setIntakeVoltage(volts));
    }

    public void perodic() {}
}
