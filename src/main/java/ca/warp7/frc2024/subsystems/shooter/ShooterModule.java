package ca.warp7.frc2024.subsystems.shooter;

import ca.warp7.frc2024.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class ShooterModule {
    private final ShooterModuleIO shooterModuleIO;
    private final int shooterModuleID;
    private final String shooterModuleName;

    private SimpleMotorFeedforward feedforward;

    private final ShooterModuleIOInputsAutoLogged shooterModuleInputs = new ShooterModuleIOInputsAutoLogged();

    public ShooterModule(ShooterModuleIO shooterModuleIO, int shooterModuleID, String shooterModuleName) {
        this.shooterModuleIO = shooterModuleIO;
        this.shooterModuleID = shooterModuleID;
        this.shooterModuleName = shooterModuleName;

        switch (Constants.CURRENT_MODE) {
            case REAL:
                feedforward = new SimpleMotorFeedforward(0.021949, 0.010201, 0.0010146);
                shooterModuleIO.configurePID(0.00065, 0.0, 0.01);
                break;
            case SIM:
                feedforward = new SimpleMotorFeedforward(0, 0.0);
                break;
            default:
                feedforward = new SimpleMotorFeedforward(0, 0, 0);
                break;
        }
    }

    public void periodic() {
        shooterModuleIO.updateInputs(shooterModuleInputs);
        Logger.processInputs("Shooter/" + shooterModuleName, shooterModuleInputs);
    }

    /**
     * @return Shooter velocity in RPM converted from radians per second
     */
    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(shooterModuleInputs.shooterVelocityRadPerSec);
    }

    /**
     * Set closed loop control target velocity
     * @param velocityRPM
     */
    public void runShooterTargetVelocity(double velocityRPM) {
        double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        shooterModuleIO.setVelocity(velocityRadPerSec, feedforward.calculate(velocityRadPerSec));

        Logger.recordOutput("Shooter/" + shooterModuleName + "/TargetRPM", velocityRPM);
    }

    public void configurePID(double kP, double kI, double kD) {
        shooterModuleIO.configurePID(kP, kI, kD);
    }

    /**
     * Set open loop control volts
     * @param volts
     */
    public void runShooterVolts(double volts) {
        shooterModuleIO.setVoltage(volts);
    }

    /**
     * Stop the shooter
     */
    public void stopShooter() {
        shooterModuleIO.stop();
    }

    /**
     * Zero shooter encoder
     */
    public void zeroEncoder() {
        shooterModuleIO.zeroEncoder();
    }
}
