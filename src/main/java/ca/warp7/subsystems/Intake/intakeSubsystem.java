package ca.warp7.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSubsystem extends SubsystemBase{
    private final intakeIO io;
    /** Creates a new intake */
    public intakeSubsystem(intakeIO io){
        this.io = io; 
    }
    public Command setIntakeRollerVoltageCommand(double volts){
        return runOnce(() -> io.setIntakeVoltage(volts));
    }

    public void perodic(){
    }
}
