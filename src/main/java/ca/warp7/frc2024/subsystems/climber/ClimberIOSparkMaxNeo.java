package ca.warp7.frc2024.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ClimberIOSparkMaxNeo implements ClimberIO {
    private final CANSparkMax motor;
    private final RelativeEncoder intEncoder;

    public ClimberIOSparkMaxNeo(int climberSparkMaxId) {
        motor = new CANSparkMax(climberSparkMaxId, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setCANTimeout(250);
        motor.enableVoltageCompensation(12);
        motor.setSmartCurrentLimit(40);
        motor.setInverted(false);
        motor.burnFlash();

        intEncoder = motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberInternalPosition = Rotation2d.fromRotations(intEncoder.getPosition());
        inputs.climberExternalPosition = new Rotation2d();
        inputs.climberInternalVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(intEncoder.getVelocity());
        inputs.climberAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.climberCurrentAmps = motor.getOutputCurrent();
        inputs.climberTempCelsius = motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
