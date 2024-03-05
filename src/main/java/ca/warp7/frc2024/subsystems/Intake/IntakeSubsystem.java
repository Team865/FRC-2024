package ca.warp7.frc2024.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    private boolean hasGamepiece;

    @Setter
    private double volts;

    /** Creates a new intake */
    public IntakeSubsystem(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    public Command runVolts(double volts) {
        return Commands.startEnd(() -> setVolts(volts), () -> this.volts = 0, this);
    }

    public boolean getSensor() {
        return intakeInputs.intakeSensorTriggered;
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Intake", intakeInputs);

        intakeIO.setVoltage(volts);
    }
}
