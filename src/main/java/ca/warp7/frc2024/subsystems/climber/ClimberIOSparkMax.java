package ca.warp7.frc2024.subsystems.climber;

import static ca.warp7.frc2024.util.SparkMaxManager.safeBurnSparkMax;
import static ca.warp7.frc2024.util.SparkMaxManager.safeSparkMax;

import ca.warp7.frc2024.Constants.CLIMBER.STATE;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ClimberIOSparkMax implements ClimberIO {
    private final CANSparkMax motor;
    private final RelativeEncoder intEncoder;

    public ClimberIOSparkMax(int climberSparkMaxId) {
        /* Create hardware objects */
        motor = new CANSparkMax(climberSparkMaxId, MotorType.kBrushless);
        intEncoder = motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

        /* Factory reset SparkMax */
        safeSparkMax(motor, motor::restoreFactoryDefaults);

        /* Configure motor invert */
        motor.setInverted(false);

        /* Configure electrical */
        safeSparkMax(motor, () -> motor.setSmartCurrentLimit(40));
        safeSparkMax(motor, () -> motor.enableVoltageCompensation(12.0));
        safeSparkMax(motor, () -> motor.setIdleMode(IdleMode.kBrake));

        /* Configure soft limits */
        motor.setSoftLimit(SoftLimitDirection.kForward, (float) STATE.CLIMBER_END.getStatePosition());

        /* Save configurations */
        safeBurnSparkMax(motor);
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
