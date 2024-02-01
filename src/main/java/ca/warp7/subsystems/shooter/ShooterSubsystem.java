package ca.warp7.subsystems.shooter;

import ca.warp7.subsystems.shooter.ShooterIO.ShooterIOInputs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    public Command setFeederRollersVoltage(double volts) {
        return runOnce(() -> io.setFeederRollersVoltage(volts));
    }

    public Command setOutrunnersVoltage(double volts) {
        return runOnce(() -> io.setOutrunnersVoltage(volts));
    }

    @Override
    public void periodic() {
        io.updateInputs(this.inputs);
        io.periodic();

        Logger.processInputs("Shooter/topFeederRoller", inputs.topFeederRoller);
        Logger.processInputs("Shooter/bottomFeederRoller", inputs.bottomFeederRoller);
        Logger.processInputs("Shooter/topRightOutrunner", inputs.topRightOutrunner);
        Logger.processInputs("Shooter/bottomRightOutrunner", inputs.bottomRightOutrunner);
        Logger.processInputs("Shooter/topLeftOutrunner", inputs.topLeftOutrunner);
        Logger.processInputs("Shooter/bottomLeftOutrunner", inputs.bottomLeftOutrunner);
    }
}
