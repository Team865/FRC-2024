package ca.warp7.frc2024.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import ca.warp7.frc2024.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    public Command setOutrunnersVoltage(double volts) {
        return runOnce(() -> io.setOutrunnersVoltage(volts));
    }

    @Override
    public void periodic() {
        io.updateInputs(this.inputs);
        io.periodic();

        Logger.processInputs("Shooter/topRightOutrunner", inputs.topRightOutrunner);
        Logger.processInputs("Shooter/bottomRightOutrunner", inputs.bottomRightOutrunner);
        Logger.processInputs("Shooter/topLeftOutrunner", inputs.topLeftOutrunner);
        Logger.processInputs("Shooter/bottomLeftOutrunner", inputs.bottomLeftOutrunner);
    }
}
