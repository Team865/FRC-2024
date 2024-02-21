package ca.warp7.frc2024.subsystems.shooter;

import ca.warp7.frc2024.Constants;
import ca.warp7.frc2024.subsystems.shooter.ShooterIO.ShooterIOInputs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();
    private final PIDController topRightFeedback;
    private final PIDController topLeftFeedback;
    private final PIDController bottomRightFeedback;
    private final PIDController bottomLeftFeedback;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;

        switch (Constants.CURRENT_MODE) {
            case REAL:
            case SIM:
                topRightFeedback = new PIDController(0.002375, 0.0, 0.0);
                topLeftFeedback = new PIDController(0.002375, 0.0, 0.0);
                bottomRightFeedback = new PIDController(0.002375, 0.0, 0.0);
                bottomLeftFeedback = new PIDController(0.002375, 0.0, 0.0);
                break;
            default:
                topRightFeedback = new PIDController(0.0, 0.0, 0.0);
                topLeftFeedback = new PIDController(0.0, 0.0, 0.0);
                bottomRightFeedback = new PIDController(0.0, 0.0, 0.0);
                bottomLeftFeedback = new PIDController(0.0, 0.0, 0.0);
                break;
        }
    }

    public Command setOutrunnersVelocity(double Velocity) {
        return runOnce(() -> {
            this.topRightFeedback.setSetpoint(Velocity);
            this.topLeftFeedback.setSetpoint(Velocity);
            this.bottomRightFeedback.setSetpoint(Velocity);
            this.bottomLeftFeedback.setSetpoint(Velocity);
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(this.inputs);

        Logger.processInputs("Shooter/topRight", inputs.topRight);
        Logger.processInputs("Shooter/topLeft", inputs.topLeft);
        Logger.processInputs("Shooter/bottomRight", inputs.bottomRight);
        Logger.processInputs("Shooter/bottomLeft", inputs.bottomLeft);

        io.setTopRightVoltage(this.topRightFeedback.calculate(inputs.topRight.VelocityRPM));
        io.setTopLeftVoltage(this.topLeftFeedback.calculate(inputs.topLeft.VelocityRPM));
        io.setBottomRightVoltage(this.bottomRightFeedback.calculate(inputs.bottomRight.VelocityRPM));
        io.setBottomLeftVoltage(this.bottomLeftFeedback.calculate(inputs.bottomLeft.VelocityRPM));
    }
}
