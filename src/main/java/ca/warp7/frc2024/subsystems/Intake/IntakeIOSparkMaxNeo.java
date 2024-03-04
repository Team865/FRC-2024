package ca.warp7.frc2024.subsystems.Intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSparkMaxNeo implements IntakeIO {
    private final CANSparkMax intakeSparkMax;

    private final RelativeEncoder intakeInternalEncoder;

    public IntakeIOSparkMaxNeo(int kIntakeNeoID) {
        /*Create Intake Neo */
        intakeSparkMax = new CANSparkMax(kIntakeNeoID, MotorType.kBrushless);
        intakeSparkMax.setCANTimeout(250);
        intakeSparkMax.enableVoltageCompensation(12.0);
        intakeSparkMax.setIdleMode(IdleMode.kBrake);
        intakeSparkMax.setSmartCurrentLimit(50);

        intakeInternalEncoder = intakeSparkMax.getEncoder();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(intakeInternalEncoder.getVelocity());
        inputs.intakeAppliedVolts = intakeSparkMax.getAppliedOutput() * intakeSparkMax.getBusVoltage();
        inputs.intakeCurrentAmps = intakeSparkMax.getOutputCurrent();
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeSparkMax.setVoltage(volts);

        Logger.recordOutput("Intake/SetVolts", volts);
    }
}
