package ca.warp7.frc2024.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean gyroConnected = false;
        public Rotation2d gyroYaw = new Rotation2d();
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void zeroYaw() {}
}
