package ca.warp7.frc2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        public Pose2d blueOriginRobotPose;
        public double timestamp;
        public double latency;
        public int tagCount;
        public double tagSpan;
        public double avgTagDist;
        public double avgTagArea;
        public String name;
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void setPipeline(int index) {}

    public default void setStreamStandard() {}

    public default void setStreamPiPMain() {}

    public default void setStreamPiPSecondary() {}
}
