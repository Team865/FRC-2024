package ca.warp7.frc2024.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {
    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    public FeederSubsystem(FeederIO feederIO) {
        this.io = feederIO;
    }

    public Command runVoltageCommand(double volts) {
        return this.runOnce(() -> io.setVoltage(volts));
    }

    public Command runVoltageCommandEnds(double volts) {
        return this.startEnd(() -> io.setVoltage(volts), () -> io.stop());
    }

    public Trigger sensorTrigger() {
        return new Trigger(() -> inputs.feederSensorTriggered).debounce(0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }
}
