package ca.warp7.frc2024.subsystems.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class IntakeIOSparkMax implements IntakeIO{
    private final CANSparkMax intakeSparkmax;

    public IntakeIOSparkMax(int intakeNeoID) {
        /*Create Intake Neo */
        intakeSparkmax = new CANSparkMax(intakeNeoID, MotorType.kBrushless);
        intakeSparkmax.restoreFactoryDefaults();
        intakeSparkmax.setCANTimeout(0);
        intakeSparkmax.setSmartCurrentLimit(20);
    }

}
