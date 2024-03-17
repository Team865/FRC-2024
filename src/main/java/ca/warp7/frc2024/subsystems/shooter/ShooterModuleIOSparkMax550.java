package ca.warp7.frc2024.subsystems.shooter;

import static ca.warp7.frc2024.util.SparkMaxManager.safeBurnSparkMax;
import static ca.warp7.frc2024.util.SparkMaxManager.safeSparkMax;

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
        /* Create hardware objects */
        motor = new CANSparkMax(shooterSparkMaxId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        feedback = motor.getPIDController();

        /* Factory reset SparkMax */
        safeSparkMax(motor, motor::restoreFactoryDefaults);

        /* Configure motor invert */
        motor.setInverted(invert);

        /* Configure electrical */
        safeSparkMax(motor, () -> motor.setSmartCurrentLimit(20));
        safeSparkMax(motor, () -> motor.enableVoltageCompensation(12.0));
        safeSparkMax(motor, () -> motor.setIdleMode(IdleMode.kBrake));

        /* Save configurations */
        safeBurnSparkMax(motor);

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
        safeSparkMax(motor, () -> feedback.setP(kP, 0));
        safeSparkMax(motor, () -> feedback.setI(kI, 0));
        safeSparkMax(motor, () -> feedback.setD(kD, 0));
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
