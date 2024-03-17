package ca.warp7.frc2024.subsystems.feeder;

import static ca.warp7.frc2024.util.SparkMaxManager.safeBurnSparkMax;
import static ca.warp7.frc2024.util.SparkMaxManager.safeSparkMax;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class FeederIOSparkMax implements FeederIO {
    /* Hardware */
    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;
    private RelativeEncoder encoder;
    private DigitalInput sensor;

    public FeederIOSparkMax(int topID, int bottomID, int photoSensorID) {
        /* Create hardware object  */
        topMotor = new CANSparkMax(topID, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(bottomID, MotorType.kBrushless);
        encoder = topMotor.getEncoder();
        sensor = new DigitalInput(photoSensorID);

        /* Factory reset SparkMaxes */
        safeSparkMax(topMotor, topMotor::restoreFactoryDefaults);
        safeSparkMax(bottomMotor, bottomMotor::restoreFactoryDefaults);

        /* Configure motor invert and follows */
        topMotor.setInverted(true);
        safeSparkMax(bottomMotor, () -> bottomMotor.follow(topMotor, false));

        /* Configure electrical */
        safeSparkMax(topMotor, () -> topMotor.setSmartCurrentLimit(20));
        safeSparkMax(bottomMotor, () -> bottomMotor.setSmartCurrentLimit(20));
        safeSparkMax(topMotor, () -> topMotor.enableVoltageCompensation(12.0));
        safeSparkMax(bottomMotor, () -> bottomMotor.enableVoltageCompensation(12.0));
        safeSparkMax(topMotor, () -> topMotor.setIdleMode(IdleMode.kBrake));
        safeSparkMax(bottomMotor, () -> bottomMotor.setIdleMode(IdleMode.kBrake));

        /* Save configurations */
        safeBurnSparkMax(topMotor);
        safeBurnSparkMax(bottomMotor);

        encoder.setPosition(0);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.feederPosition = Rotation2d.fromRotations(encoder.getPosition());
        inputs.feederVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        inputs.feederAppliedVolts = new double[] {
            topMotor.getAppliedOutput() * topMotor.getBusVoltage(),
            bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage()
        };
        inputs.feederCurrentAmps = new double[] {topMotor.getOutputCurrent(), bottomMotor.getOutputCurrent()};
        inputs.feederTempCelsius = new double[] {topMotor.getMotorTemperature(), bottomMotor.getMotorTemperature()};
        inputs.feederSensorTriggered = sensor.get();
    }

    @Override
    public void setVoltage(double volts) {
        topMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        topMotor.stopMotor();
    }
}
