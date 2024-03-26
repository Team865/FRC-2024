package ca.warp7.frc2024.subsystems.shooter;

import static ca.warp7.frc2024.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterSubsystemCommands extends ShooterSubsystem {

    public ShooterSubsystemCommands(
            ShooterModuleIO topRightShooterModuleIO,
            ShooterModuleIO topLeftShooterModuleIO,
            ShooterModuleIO bottomLeftShooterModuleIO,
            ShooterModuleIO bottomRightShooterModuleIO) {
        super(topRightShooterModuleIO, topLeftShooterModuleIO, bottomLeftShooterModuleIO, bottomRightShooterModuleIO);
    }

    public Command runVelocityCommand(double RPM, int... shooterModules) {
        return super.runOnce(() -> setVelocity(RPM, shooterModules));
    }

    public Command runVelocityCommandEnds(double RPM, int... shooterModules) {
        return super.startEnd(() -> setVelocity(RPM, shooterModules), () -> stopShooter());
    }

    public Command runVoltageCommand(double volts, int... shooterModules) {
        return super.runOnce(() -> setVoltage(volts, shooterModules));
    }

    public Command runVoltageCommandEnds(double volts, int... shooterModules) {
        return super.startEnd(() -> setVoltage(volts, shooterModules), () -> stopShooter());
    }

    public Command runGoalCommand(Goal goal) {
        return super.runOnce(() -> setGoal(goal));
    }

    public Command runGoalCommandEnds(Goal goal) {
        return super.startEnd(() -> setGoal(goal), () -> stopShooter());
    }

    public Command stopShooterCommand() {
        return super.runOnce(() -> stopShooter());
    }

    public Command zeroEncoderCommand() {
        return super.runOnce(() -> zeroEncoder());
    }

    /**
     * Shooter quasistatic SysID routine
     * @param direction
     * @return
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /**
     * Shooter dynamic SysID routine
     * @param direction
     * @return
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }
}
