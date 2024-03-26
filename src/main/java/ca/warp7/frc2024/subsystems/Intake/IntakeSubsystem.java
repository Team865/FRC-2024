package ca.warp7.frc2024.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSubsystem(IntakeIO intakeIO) {
        this.io = intakeIO;
    }

    public Command runVoltageCommand(double volts) {
        return this.runOnce(() -> io.setVoltage(volts));
    }

    public Command runVoltageCommandEnds(double volts) {
        return this.startEnd(() -> io.setVoltage(volts), () -> io.stop());
    }

    public Trigger sensorTrigger() {
        return new Trigger(() -> inputs.intakeSensorTriggered).debounce(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
