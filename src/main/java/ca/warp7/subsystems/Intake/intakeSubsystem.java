package ca.warp7.subsystems.Intake;

import static ca.warp7.Constants.kIntake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSubsystem extends SubsystemBase{
    private final CANSparkMax intakeMoter;
    



    public intakeSubsystem() {
        intakeMoter = new CANSparkMax(kIntake.kIntakeMotorID, MotorType.kBrushless);
        intakeMoter.setSmartCurrentLimit(20);
        intakeMoter.setInverted(true);
    }
}
