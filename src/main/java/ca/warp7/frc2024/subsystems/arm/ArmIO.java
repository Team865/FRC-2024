package ca.warp7.frc2024.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public Rotation2d armPosition = new Rotation2d();
        public Rotation2d armAbsolutePosition = new Rotation2d();
        public double armVelocityRadPerSec = 0.0;
        public double armAppliedVolts = 0.0;
        public double[] armCurrentAmps = new double[] {};
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setArmVoltage(double volts) {}
}
