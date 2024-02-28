package ca.warp7.frc2024.subsystems.Intake;

import ca.warp7.frc2024.Constants.kIntake;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class IntakeIOSparkMaxNeo implements IntakeIO {
    private final CANSparkMax intakeSparkmax;

    public IntakeIOSparkMaxNeo(int kIntakeNeoID) {
        /*Create Intake Neo */
        intakeSparkmax = new CANSparkMax(kIntake.kIntakeNeoID, MotorType.kBrushless);
        intakeSparkmax.restoreFactoryDefaults();
        intakeSparkmax.setCANTimeout(0);
        intakeSparkmax.setSmartCurrentLimit(20);
    }
}
