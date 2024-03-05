package ca.warp7.frc2024.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class ArmIOSparkMax implements ArmIO {
    /* Hardware */
    private final CANSparkMax rightMotor; // Primary motor
    private final CANSparkMax leftMotor; // Secondary motor
    private final RelativeEncoder intIncEncoder; // NEO Hall
    private final Encoder extIncEncoder; // REV Through Bore encoder
    private final DutyCycleEncoder extAbsEncoder; // REV Through Bore encoder

    /* Constants */
    private final double ARM_RATIO = 1.0 / (20.0 * (74.0 / 22.0));
    private final double ENCODER_RATIO = (18.0 / 42.0);
    private final double EXTERNAL_ENCODER_CPR = 2048.0;

    private final Rotation2d dutyCycleEncOffset;

    /**
     *
     * @param armLeftID Arm left SparkMax CAN ID
     * @param armRightID Arm right SparkMax CAN ID
     * @param extIncEncChannelA External incremental encoder channel A
     * @param extIncEncChannelB External incremental encoder channel B
     * @param extAbsEncChannel External absolute encoder channel
     */
    public ArmIOSparkMax(
            int armRightNeoID,
            int armLeftNeoID,
            int extIncEncChannelA,
            int extIncEncChannelB,
            int dutyCycleEncChannel,
            Rotation2d dutyCycleEncOffset) {
        /* Create hardware objects */
        rightMotor = new CANSparkMax(armRightNeoID, MotorType.kBrushless);
        leftMotor = new CANSparkMax(armLeftNeoID, MotorType.kBrushless);
        intIncEncoder = rightMotor.getEncoder();
        extIncEncoder = new Encoder(extIncEncChannelA, extIncEncChannelB, true, Encoder.EncodingType.k4X);
        extAbsEncoder = new DutyCycleEncoder(dutyCycleEncChannel);

        /* Factory reset SparkMaxes */
        rightMotor.restoreFactoryDefaults();
        leftMotor.restoreFactoryDefaults();

        /* Configure motor invert and follows */
        rightMotor.setInverted(false);
        leftMotor.follow(rightMotor, true); // Set the left motor to follow the right motor

        /* Configure electrical */
        rightMotor.setSmartCurrentLimit(40);
        leftMotor.setSmartCurrentLimit(40);
        rightMotor.enableVoltageCompensation(12.0);
        leftMotor.enableVoltageCompensation(12.0);
        rightMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setIdleMode(IdleMode.kBrake);

        /* Configure status frames */
        // While we are using an external encoder, we rely on the internal encoder as a fallback if the external encoder
        // fails thus we do not want to reduce that frame
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500); // No positional telemetry from follower motor

        /* Configure encoders */
        intIncEncoder.setPosition(0.0);
        extIncEncoder.reset();
        extAbsEncoder.reset();

        this.dutyCycleEncOffset = dutyCycleEncOffset;

        /* Save configuration */
        rightMotor.burnFlash();
        leftMotor.burnFlash();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armInternalIncrementalPosition = Rotation2d.fromRotations(intIncEncoder.getPosition() * ARM_RATIO);
        inputs.armExternalIncrementalPosition =
                Rotation2d.fromRotations(-1.0 * (extIncEncoder.getDistance() / EXTERNAL_ENCODER_CPR * ENCODER_RATIO));
        inputs.armExternalAbsolutePosition = Rotation2d.fromRotations(
                -1.0 * ((extAbsEncoder.getDistance() * ENCODER_RATIO) - dutyCycleEncOffset.getRotations()));

        inputs.armInternalVelocityRadPerSec = intIncEncoder.getVelocity() * ARM_RATIO;
        inputs.armExternalVelocityRadPerSec = extIncEncoder.getRate() / EXTERNAL_ENCODER_CPR * ENCODER_RATIO;

        inputs.armAppliedVolts = new double[] {
            rightMotor.getAppliedOutput() * rightMotor.getBusVoltage(),
            leftMotor.getAppliedOutput() * leftMotor.getBusVoltage()
        };
        inputs.armCurrentAmps = new double[] {rightMotor.getOutputCurrent(), leftMotor.getOutputCurrent()};
        inputs.armTempCelsius = new double[] {rightMotor.getMotorTemperature(), leftMotor.getMotorTemperature()};
    }

    @Override
    public void setVoltage(double volts) {
        rightMotor.setVoltage(volts);
    }

    @Override
    public void stopArm() {
        rightMotor.stopMotor();
    }
}
