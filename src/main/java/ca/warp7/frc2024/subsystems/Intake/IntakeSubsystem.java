package ca.warp7.frc2024.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    @Setter
    private double volts = 0;

    public IntakeSubsystem(IntakeIO intakeIO) {
        this.io = intakeIO;
    }

    private boolean getSensor() {
        return inputs.intakeSensorTriggered;
    }

    public Command runVoltage(double volts) {
        return this.startEnd(() -> setVolts(volts), () -> setVolts(0));
    }

    public Trigger sensorTrigger() {
        return new Trigger(() -> getSensor()).debounce(0.075);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        io.setVoltage(volts);
    }
}
