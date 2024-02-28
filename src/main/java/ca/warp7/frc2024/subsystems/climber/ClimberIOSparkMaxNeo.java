package ca.warp7.frc2024.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ClimberIOSparkMaxNeo implements ClimberIO {
    private final CANSparkMax climberSparkMax;
    private final RelativeEncoder climberInternalEncoder;

    public ClimberIOSparkMaxNeo(int climberSparkMaxId) {
        climberSparkMax = new CANSparkMax(climberSparkMaxId, MotorType.kBrushless);

        climberSparkMax.restoreFactoryDefaults();
        climberSparkMax.setCANTimeout(250);
        climberSparkMax.enableVoltageCompensation(12);
        climberSparkMax.setSmartCurrentLimit(40);
        climberSparkMax.setInverted(false);
        climberSparkMax.burnFlash();

        climberInternalEncoder = climberSparkMax.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberInternalPositionRad = Units.rotationsToRadians(climberInternalEncoder.getPosition());
        inputs.climberVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(climberInternalEncoder.getVelocity());
        inputs.climberExternalPositionRad = new Rotation2d();
        inputs.climberAppliedVolts = climberSparkMax.getAppliedOutput() * climberSparkMax.getBusVoltage();
        inputs.climberCurrentAmps = climberSparkMax.getOutputCurrent();
    }

    @Override
    public void setClimberVoltage(double volts) {
        climberSparkMax.setVoltage(volts);
    }
}
