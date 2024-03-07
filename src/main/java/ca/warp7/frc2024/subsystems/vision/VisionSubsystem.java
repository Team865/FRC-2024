package ca.warp7.frc2024.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;

    public VisionSubsystem(VisionIO[] io) {
        this.io = io;
        inputs = new VisionIOInputsAutoLogged[io.length];

        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs(inputs[i].name, inputs[i]);
        }
    }
}
