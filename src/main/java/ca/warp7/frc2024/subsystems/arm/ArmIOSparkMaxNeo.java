package ca.warp7.frc2024.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

public class ArmIOSparkMaxNeo implements ArmIO {
    private final CANSparkMax armLeftSparkMax; // Secondary motor
    private final CANSparkMax armRightSparkMax; // Primary motor

    private final RelativeEncoder armRightEncoder;
    private final SparkAbsoluteEncoder armAbsoluteEncoder;

    public ArmIOSparkMaxNeo(int armLeftNeoID, int armRightNeoID) {
        // Create neo objects
        armLeftSparkMax = new CANSparkMax(armLeftNeoID, MotorType.kBrushless);
        armRightSparkMax = new CANSparkMax(armRightNeoID, MotorType.kBrushless);

        armLeftSparkMax.restoreFactoryDefaults();
        armRightSparkMax.restoreFactoryDefaults();

        armLeftSparkMax.setCANTimeout(250);
        armRightSparkMax.setCANTimeout(250);

        // Set the left motor to follow the right motor
        armLeftSparkMax.follow(armRightSparkMax, true);

        armRightSparkMax.enableVoltageCompensation(12.0);
        armRightSparkMax.setSmartCurrentLimit(40);

        armRightSparkMax.burnFlash();
        armLeftSparkMax.burnFlash();

        // Create encoder objects
        armRightEncoder = armRightSparkMax.getEncoder();
        armAbsoluteEncoder = armRightSparkMax.getAbsoluteEncoder();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {}
}
