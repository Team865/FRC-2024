package ca.warp7.frc2024.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import ca.warp7.frc2024.subsystems.feeder.FeederIO.FeederIOInputs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final FeederIO io;
    private final FeederIOInputs inputs = new FeederIOInputs();

    public FeederSubsystem(FeederIO io) {
        this.io = io;
    }

    public Command setFeederRollersVoltage(double volts) {
        return runOnce(() -> io.setFeederRollersVoltage(volts));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.periodic();

        Logger.processInputs("Feeder/topFeederRoller", inputs.topFeederRoller);
        Logger.processInputs("Feeder/bottomFeederRoller", inputs.bottomFeederRoller);
    }
}
