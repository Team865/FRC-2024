package ca.warp7.frc2024.subsystems.feeder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
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
        topMotor = new CANSparkMax(topID, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(bottomID, MotorType.kBrushless);
        encoder = topMotor.getEncoder();
        sensor = new DigitalInput(photoSensorID);

        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        topMotor.setInverted(true);
        bottomMotor.follow(topMotor, false);

        topMotor.setSmartCurrentLimit(15);
        bottomMotor.setSmartCurrentLimit(15);
        topMotor.enableVoltageCompensation(12);
        bottomMotor.enableVoltageCompensation(12);
        topMotor.setIdleMode(IdleMode.kBrake);
        bottomMotor.setIdleMode(IdleMode.kBrake);

        bottomMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

        encoder.setPosition(0);

        topMotor.burnFlash();
        bottomMotor.burnFlash();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.feederPosition = Rotation2d.fromRotations(encoder.getPosition());
        inputs.feederVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        inputs.feederAppliedVolts = topMotor.getAppliedOutput() * topMotor.getBusVoltage();
        inputs.feederCurrentAmps = new double[] {topMotor.getOutputCurrent(), bottomMotor.getOutputCurrent()};
        inputs.feederSensorTriggered = sensor.get();
    }

    @Override
    public void setFeederVoltage(double volts) {
        topMotor.setVoltage(volts);
    }
}
