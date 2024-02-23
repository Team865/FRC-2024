package ca.warp7.frc2024.subsystems.feeder;

import ca.warp7.frc2024.Constants;
import ca.warp7.frc2024.subsystems.feeder.FeederIO.FeederIOInputs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {
    private final FeederIO io;
    private final FeederIOInputs inputs = new FeederIOInputs();
    private final PIDController topFeedback;
    private final PIDController bottomFeedback;

    public FeederSubsystem(FeederIO io) {
        this.io = io;

        switch (Constants.CURRENT_MODE) {
            case REAL:
            case SIM:
                topFeedback = new PIDController(0.0, 0.0, 0.0);
                bottomFeedback = new PIDController(0.0, 0.0, 0.0);
                break;
            default:
                topFeedback = new PIDController(0.0, 0.0, 0.0);
                bottomFeedback = new PIDController(0.0, 0.0, 0.0);
                break;
        }
    }

    public Command setRollersVelocity(double velocity) {
        return runOnce(() -> {
            topFeedback.setSetpoint(velocity);
            bottomFeedback.setSetpoint(velocity);
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("Feeder/topFeederRoller", inputs.topRoller);
        Logger.processInputs("Feeder/bottomFeederRoller", inputs.bottomRoller);

        io.setTopVoltage(topFeedback.calculate(inputs.topRoller.VelocityRPM));
        io.setTopVoltage(bottomFeedback.calculate(inputs.bottomRoller.VelocityRPM));
    }
}
