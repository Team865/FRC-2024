package ca.warp7.frc2024.subsystems.arm;

import static ca.warp7.frc2024.subsystems.arm.ArmConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ArmSubsystemCommands extends ArmSubsystem {

    public ArmSubsystemCommands(ArmIO armIO) {
        super(armIO);
    }

    public Trigger atGoalTrigger(Goal goal) {
        return new Trigger(() -> atGoal(goal)).debounce(0.1);
    }

    public Command runGoalCommand(Goal goal) {
        return super.runOnce(() -> super.currentGoal = goal);
    }

    public Command runGoalCommandUntil(Goal goal) {
        return super.runOnce(() -> super.currentGoal = goal).until(atGoalTrigger(goal));
    }
    
    public Command setLockoutCommand(boolean lockedOut) {
        return super.runOnce(() -> super.lockout = lockedOut);
    }

    public Command setDistance(double distance) {
        return super.runOnce(() -> ArmSubsystem.distance = distance);
    }
}
