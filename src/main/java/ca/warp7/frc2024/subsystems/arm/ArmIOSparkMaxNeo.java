package ca.warp7.frc2024.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class ArmIOSparkMaxNeo implements ArmIO {
    private final CANSparkMax armLeftSparkMax; // Secondary motor 67.27/20
    private final CANSparkMax armRightSparkMax; // Primary motor

    private final Encoder externalIncrementalEncoder;
    private final DutyCycleEncoder externalAbsoluteEncoder;

    private final RelativeEncoder internalIncrementalEncoder;

    private final double ARM_RATIO = 1 / ((20 / 1) * (74 / 22));
    private final double ENCODER_RATIO = 18 / 42;
    private final double ENCODER_CPR = 2048;

    public ArmIOSparkMaxNeo(int armLeftNeoID, int armRightNeoID) {
        // Create neo objects
        armLeftSparkMax = new CANSparkMax(armLeftNeoID, MotorType.kBrushless);
        armRightSparkMax = new CANSparkMax(armRightNeoID, MotorType.kBrushless);

        armLeftSparkMax.restoreFactoryDefaults();
        armRightSparkMax.restoreFactoryDefaults();

        armLeftSparkMax.setCANTimeout(250);
        armRightSparkMax.setCANTimeout(250);

        armRightSparkMax.setInverted(true);

        // Set the left motor to follow the right motor
        armLeftSparkMax.follow(armRightSparkMax, true);

        armRightSparkMax.enableVoltageCompensation(12.0);
        armRightSparkMax.setSmartCurrentLimit(40);

        armRightSparkMax.burnFlash();
        armLeftSparkMax.burnFlash();

        // Create encoder objects
        internalIncrementalEncoder = armRightSparkMax.getEncoder();

        externalIncrementalEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
        externalAbsoluteEncoder = new DutyCycleEncoder(2);

        externalIncrementalEncoder.setDistancePerPulse(1);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armInternalIncrementalPosition =
                Rotation2d.fromRotations(internalIncrementalEncoder.getPosition() / 4 / 42 / 20 * 74 / 22);
        inputs.armExternalIncrementalPosition =
                Rotation2d.fromRotations(externalIncrementalEncoder.getDistance() / 2048 * 18 / 42);
        inputs.armExternalAbsolutePosition = Rotation2d.fromRotations(externalAbsoluteEncoder.getDistance());

        inputs.armInternalVelocityRadPerSec = internalIncrementalEncoder.getVelocity() * ARM_RATIO;
        inputs.armExternalVelocityRadPerSec = externalIncrementalEncoder.getRate() / ENCODER_CPR * ENCODER_RATIO;
        inputs.armAppliedVolts = armRightSparkMax.getAppliedOutput() * armRightSparkMax.getBusVoltage();
        inputs.armCurrentAmps = new double[] {armRightSparkMax.getOutputCurrent(), armLeftSparkMax.getOutputCurrent()};
    }

    @Override
    public void setArmVoltage(double volts) {
        armRightSparkMax.setVoltage(volts);
    }
}
