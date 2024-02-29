package ca.warp7.frc2024.subsystems.feeder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class FeederIOSparkMax550 implements FeederIO {
    private CANSparkMax topRollerSparkMax;
    private RelativeEncoder topRollerEncoder;

    private CANSparkMax bottomRollerSparkMax;

    public FeederIOSparkMax550(int topID, int bottomID) {
        topRollerSparkMax = new CANSparkMax(topID, MotorType.kBrushless);
        bottomRollerSparkMax = new CANSparkMax(bottomID, MotorType.kBrushless);

        setupMotorControllers();

        topRollerEncoder = topRollerSparkMax.getEncoder();
    }

    private void setupMotorControllers() {
        topRollerSparkMax.restoreFactoryDefaults();
        bottomRollerSparkMax.restoreFactoryDefaults();

        topRollerSparkMax.setCANTimeout(250);
        bottomRollerSparkMax.setCANTimeout(250);

        topRollerSparkMax.enableVoltageCompensation(12.0);
        bottomRollerSparkMax.enableVoltageCompensation(12.0);

        topRollerSparkMax.setSmartCurrentLimit(15);
        bottomRollerSparkMax.setSmartCurrentLimit(15);

        topRollerSparkMax.setIdleMode(IdleMode.kBrake);
        bottomRollerSparkMax.setIdleMode(IdleMode.kBrake);

        topRollerSparkMax.setInverted(true);

        bottomRollerSparkMax.follow(topRollerSparkMax, false);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.feederVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(topRollerEncoder.getVelocity());
        inputs.feederAppliedVolts = topRollerSparkMax.getAppliedOutput() * topRollerSparkMax.getBusVoltage();
        inputs.feederCurrentAmps = topRollerSparkMax.getOutputCurrent();
    }

    @Override
    public void setFeederVoltage(double volts) {
        topRollerSparkMax.setVoltage(volts);
    }
}
