package ca.warp7.frc2024.subsystems.shooter;

import ca.warp7.frc2024.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLogOutput;
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
                feedforward = new SimpleMotorFeedforward(0.045574, 0.010104, 0.0012135);
                shooterModuleIO.configureShooterPID(0.00045, 0.0, 0.004);
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
    @AutoLogOutput(key = "Shooter/{shooterModuleName}/RealRPM")
    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(shooterModuleInputs.shooterVelocityRadPerSec);
    }

    /**
     * Set closed loop control target velocity
     * @param velocityRPM
     */
    public void runShooterTargetVelocity(double velocityRPM) {
        double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        shooterModuleIO.runShooterVelocity(velocityRadPerSec, feedforward.calculate(velocityRadPerSec));

        Logger.recordOutput("Shooter/" + shooterModuleName + "/TargetRPM", velocityRPM);
    }

    /**
     * Set open loop control volts
     * @param volts
     */
    public void runShooterVolts(double volts) {
        shooterModuleIO.runShooterVolts(volts);
    }

    /**
     * Stop the shooter
     */
    public void stopShooter() {
        shooterModuleIO.stopShooter();
    }

    /**
     * Zero shooter encoder
     */
    public void zeroEncoder() {
        shooterModuleIO.zeroEncoder();
    }
}
