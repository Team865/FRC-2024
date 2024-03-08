package ca.warp7.frc2024.subsystems.climber;

import ca.warp7.frc2024.Constants.CLIMBER.STATE;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private boolean lockedOut;
    private STATE currentState;

    public ClimberSubsystem(ClimberIO climberIO) {
        this.io = climberIO;

        // Set default startup configuration
        lockedOut = true;
        currentState = STATE.CLIMBER_START;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        if (getPosition() >= STATE.CLIMBER_END.getStatePosition()) {
            currentState = STATE.CLIMBER_END;
        } else if (getPosition() > STATE.CLIMBER_END_HIGHEST.getStatePosition()) {
            currentState = STATE.CLIMBER_END_HIGHEST;
        } else if (getPosition() > STATE.CLIMBER_START_HIGHEST.getStatePosition()) {
            currentState = STATE.CLIMBER_START_HIGHEST;
        }
    }

    public double getPosition() {
        return inputs.climberInternalPosition.getRotations();
    }

    public void runVoltage(double volts) {
        io.setVoltage(volts);
    }

    public Trigger climberLockoutDisabledTrigger() {
        return new Trigger(() -> !lockedOut);
    }

    public Command setClimberLockout(boolean lockoutEnabled) {
        return this.runOnce(() -> lockedOut = lockoutEnabled);
    }

    public Command toggleClimberLockout() {
        return this.runOnce(() -> lockedOut = !lockedOut);
    }

    public static Command climberCommand(ClimberSubsystem climberSubsystem, DoubleSupplier input) {
        return Commands.run(
                () -> {
                    double voltage =
                            MathUtil.clamp(Math.abs(MathUtil.applyDeadband(input.getAsDouble(), 0.1) * 12), 0, 12);
                    climberSubsystem.runVoltage(voltage);
                },
                climberSubsystem);
    }
}
