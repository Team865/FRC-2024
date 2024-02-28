package ca.warp7.frc2024.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO climberIO;
    private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    public ClimberSubsystem(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);
        Logger.processInputs("Climber", climberInputs);
    }

    public void setClimberVoltage(double volts) {
        climberIO.setClimberVoltage(volts);
    }

    public static Command climberCommand(ClimberSubsystem climberSubsystem, DoubleSupplier input) {
        return Commands.run(
                () -> {
                    double voltage =
                            MathUtil.clamp(Math.abs(MathUtil.applyDeadband(input.getAsDouble(), 0.1) * 12), -12, 12);
                    climberSubsystem.setClimberVoltage(voltage);
                },
                climberSubsystem);
    }
}
