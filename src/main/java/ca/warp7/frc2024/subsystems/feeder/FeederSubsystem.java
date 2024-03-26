package ca.warp7.frc2024.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {
    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    @Setter
    private double volts = 0;

    public FeederSubsystem(FeederIO feederIO) {
        this.io = feederIO;
    }

    private boolean getSensor() {
        return inputs.feederSensorTriggered;
    }

    public Command runVoltage(double volts) {
        return this.startEnd(() -> setVolts(volts), () -> setVolts(0));
    }

    public Trigger sensorTrigger() {
        return new Trigger(() -> getSensor()).debounce(0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        io.setVoltage(volts);
    }
}
