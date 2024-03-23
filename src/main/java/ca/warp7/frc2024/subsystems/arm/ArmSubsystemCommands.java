package ca.warp7.frc2024.subsystems.arm;

import static ca.warp7.frc2024.subsystems.arm.ArmConstants.*;

import ca.warp7.frc2024.subsystems.arm.ArmConstants.Goal;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class ArmSubsystemCommands extends ArmSubsystem {

    public ArmSubsystemCommands(ArmIO armIO) {
        super(armIO);
    }

    public Trigger atGoalTrigger(Goal goal) {
        return new Trigger(() -> atGoal(goal)).debounce(0);
    }

    public Command runGoalCommand(Goal goal) {
        return super.runOnce(() -> super.currentGoal = goal);
    }

    public Command runGoalCommandUntil(Goal goal) {
        return super.runOnce(() -> super.currentGoal = goal).until(atGoalTrigger(goal));
    }

    public Command setInterpolationDistanceCommand(DoubleSupplier distance) {
        return super.run(() -> super.distance = distance.getAsDouble());
    }

    public Command runInterpolation(DoubleSupplier distance) {
        return Commands.run(() -> super.distance = distance.getAsDouble())
                .beforeStarting(runGoalCommand(Goal.INTERPOLATION));
    }

    public Command setLockoutCommand(boolean lockedOut) {
        return super.runOnce(() -> super.lockout = lockedOut);
    }
}
