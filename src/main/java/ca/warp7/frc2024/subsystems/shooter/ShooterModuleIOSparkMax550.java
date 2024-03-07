package ca.warp7.frc2024.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

public class ShooterModuleIOSparkMax550 implements ShooterModuleIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    private final SparkPIDController feedback;

    public ShooterModuleIOSparkMax550(int shooterSparkMaxId, boolean invert) {
        motor = new CANSparkMax(shooterSparkMaxId, MotorType.kBrushless);

        // TODO: FIX config
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(15);
        motor.setIdleMode(IdleMode.kCoast);
        if (invert) {
            motor.setInverted(true);
        }

        // Specify encoder type: https://www.chiefdelphi.com/t/psa-new-crash-bug-in-revlib-2024-2-2/456242
        encoder = motor.getEncoder();
        feedback = motor.getPIDController();
        encoder.setAverageDepth(8);
        encoder.setMeasurementPeriod(8);
        feedback.setFeedbackDevice(encoder);
    }

    @Override
    public void updateInputs(ShooterModuleIOInputs inputs) {
        inputs.shooterPositionRad = encoder.getPosition();
        inputs.shooterVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        inputs.shooterAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.shooterCurrentAmps = motor.getOutputCurrent();
        inputs.shooterTempCelsius = motor.getMotorTemperature();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        feedback.setP(kP, 0);
        feedback.setI(kI, 0);
        feedback.setD(kD, 0);
    }

    @Override
    public void setVelocity(double velocityRadPerSec, double arbFfVolts) {
        feedback.setReference(
                Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
                ControlType.kVelocity,
                0,
                arbFfVolts,
                ArbFFUnits.kVoltage);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void zeroEncoder() {
        encoder.setPosition(0);
    }
}
