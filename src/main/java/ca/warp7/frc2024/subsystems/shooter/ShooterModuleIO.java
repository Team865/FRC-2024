package ca.warp7.frc2024.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterModuleIO {
    @AutoLog
    public static class ShooterModuleIOInputs {
        public double shooterPositionRad = 0.0;
        public double shooterVelocityRadPerSec = 0.0;
        public double shooterAppliedVolts = 0.0;
        public double shooterCurrentAmps = 0.0;
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
    public default void configureShooterPID(double kP, double kI, double kD) {}

    /**
     * Run the shooter module at specified velocity along with a voltage feedforward
     *
     * @param velocityRadPerSec
     * @param arbFfVolts
     */
    public default void runShooterVelocity(double velocityRadPerSec, double arbFfVolts) {}

    /** Run the shooter module at the specified voltage;
     *
     * @param volts
     */
    public default void runShooterVolts(double volts) {}

    /**
     * Stop the shooter
     */
    public default void stopShooter() {}

    /**
     * Zero the shooter encoder
     * Used between SysID routine runs
     */
    public default void zeroEncoder() {}
}
