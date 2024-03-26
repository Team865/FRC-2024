package ca.warp7.frc2024.subsystems.Intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO {
    /* Hardware */
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput sensor;

    public IntakeIOSparkMax(int kIntakeNeoID, int sensorID) {
        /* Create hardware object */
        motor = new CANSparkMax(kIntakeNeoID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        sensor = new DigitalInput(sensorID);

        motor.restoreFactoryDefaults();
        motor.setInverted(true);

        motor.setSmartCurrentLimit(45);
        motor.enableVoltageCompensation(12.0);
        motor.setIdleMode(IdleMode.kBrake);
        motor.burnFlash();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakePosition = Rotation2d.fromRotations(encoder.getPosition());
        inputs.intakeVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        inputs.intakeAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.intakeCurrentAmps = motor.getOutputCurrent();
        inputs.intakeTempCelsius = motor.getMotorTemperature();

        inputs.intakeSensorTriggered = sensor.get();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
