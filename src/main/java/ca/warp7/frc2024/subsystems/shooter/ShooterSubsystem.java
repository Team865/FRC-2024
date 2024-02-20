package ca.warp7.frc2024.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import ca.warp7.frc2024.Constants;
import ca.warp7.frc2024.subsystems.shooter.ShooterIO.ShooterIOInputs;

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
                topLeftFeedback = new PIDController(0.001, 0.0, 0.0);
                bottomRightFeedback = new PIDController(0.001, 0.0, 0.0);
                bottomLeftFeedback = new PIDController(0.001, 0.0, 0.0);
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

        Logger.processInputs("Shooter/topRightOutrunner", inputs.topRightOutrunner);
        Logger.processInputs("Shooter/topLeftOutrunner", inputs.topLeftOutrunner);
        Logger.processInputs("Shooter/bottomRightOutrunner", inputs.bottomRightOutrunner);
        Logger.processInputs("Shooter/bottomLeftOutrunner", inputs.bottomLeftOutrunner);

        io.setTopRightVoltage(this.topRightFeedback.calculate(inputs.topRightOutrunner.VelocityRad));
        io.setTopLeftVoltage(this.topLeftFeedback.calculate(inputs.topLeftOutrunner.VelocityRad));
        io.setbottomRightVoltage(this.bottomRightFeedback.calculate(inputs.topRightOutrunner.VelocityRad));
        io.setbottomLeftVoltage(this.bottomLeftFeedback.calculate(inputs.topLeftOutrunner.VelocityRad));
    }
}
