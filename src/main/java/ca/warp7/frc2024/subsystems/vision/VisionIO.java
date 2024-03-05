package ca.warp7.frc2024.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
