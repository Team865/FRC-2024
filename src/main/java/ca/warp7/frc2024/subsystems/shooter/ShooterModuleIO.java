package ca.warp7.frc2024.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterModuleIO {
    @AutoLog
    public static class ShooterModuleIOInputs {
        public Rotation2d shooterPositionRad = new Rotation2d();
        public double shooterVelocityRadPerSec = 0.0;
        public double shooterAppliedVolts = 0.0;
        public double shooterCurrentAmps = 0.0;
        public double shooterTempCelsius = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     *
     * @param inputs
     */
    public default void updateInputs(ShooterModuleIOInputs inputs) {}

    /**
     * Configure the shooter modules PID controller
     * @param kP
     * @param kI
     * @param kD
     */
    public default void configurePID(double kP, double kI, double kD) {}

    /**
     * Run the shooter module at specified velocity along with a voltage feedforward
     *
     * @param velocityRadPerSec
     * @param arbFfVolts
     */
    public default void setVelocity(double velocityRadPerSec, double arbFfVolts) {}

    /** Run the shooter module at the specified voltage;
     *
     * @param volts
     */
    public default void setVoltage(double volts) {}

    /**
     * Stop the shooter
     */
    public default void stop() {}

    /**
     * Zero the shooter encoder
     * Used between SysID routine runs
     */
    public default void zeroEncoder() {}
}
