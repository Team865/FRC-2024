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
                // TODO: values currently unused while testing
                feedforward = new SimpleMotorFeedforward(0.023286, 0.010337, 0.00187);
                shooterModuleIO.configureShooterPID(0.0, 0.0, 0.0);
                break;
            case SIM:
                feedforward = new SimpleMotorFeedforward(0, 0.0);
                break;
            default:
                feedforward = new SimpleMotorFeedforward(0, 0, 0);
                break;
        }
    }

    public void setVolts(double volts) {
        shooterModuleIO.setShooterVoltage(volts);
    }

    public void setTargetVelocity(double velocityRPM) {
        double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        shooterModuleIO.setShooterVelocity(velocityRadPerSec, feedforward.calculate(velocityRadPerSec));

        Logger.recordOutput("Shooter/" + shooterModuleName + "/TargetRPM", velocityRPM);
    }

    public void stopShooter() {
        shooterModuleIO.stopShooter();
    }

    @AutoLogOutput(key = "Shooter/{shooterModuleName}/RealRPM")
    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(shooterModuleInputs.shooterVelocityRadPerSec);
    }

    public void periodic() {
        shooterModuleIO.updateInputs(shooterModuleInputs);
        Logger.processInputs("Shooter/" + shooterModuleName, shooterModuleInputs);
    }

    public void zeroEncoder() {
        shooterModuleIO.zeroEncoder();
    }
}
