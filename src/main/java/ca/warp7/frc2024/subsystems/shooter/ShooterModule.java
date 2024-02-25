package ca.warp7.frc2024.subsystems.shooter;

import ca.warp7.frc2024.Constants;
import org.littletonrobotics.junction.Logger;

public class ShooterModule {
    private final ShooterModuleIO shooterModuleIO;
    private final int shooterModuleID;
    private final String shooterModuleName;

    // TODO: controllers

    private final ShooterModuleIOInputsAutoLogged shooterModuleInputs = new ShooterModuleIOInputsAutoLogged();

    public ShooterModule(ShooterModuleIO shooterModuleIO, int shooterModuleID, String shooterModuleName) {
        this.shooterModuleIO = shooterModuleIO;
        this.shooterModuleID = shooterModuleID;
        this.shooterModuleName = shooterModuleName;

        switch (Constants.CURRENT_MODE) {
            case REAL:
            case SIM:
                break;
            default:
                break;
        }
    }

    public double setTargetRPM(double RPM) {

        return 0; // TODO
    }

    public void periodic() {
        shooterModuleIO.updateInputs(shooterModuleInputs);
        Logger.processInputs("Shooter/" + shooterModuleName, shooterModuleInputs);

        double shooterVoltage = 0; // TODO: Feedforward and feedback controller
    }
}
