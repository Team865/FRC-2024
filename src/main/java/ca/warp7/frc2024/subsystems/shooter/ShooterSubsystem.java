package ca.warp7.frc2024.subsystems.shooter;

import static ca.warp7.frc2024.subsystems.shooter.ShooterConstants.*;

import ca.warp7.frc2024.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterModule[] shooterModules;

    protected final SysIdRoutine sysId;

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Gains/kP", Gains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/Gains/kI", Gains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Gains/kD", Gains.kD());

    protected Goal currentGoal = Goal.IDLE;

    public ShooterSubsystem(
            ShooterModuleIO topRightShooterModuleIO,
            ShooterModuleIO topLeftShooterModuleIO,
            ShooterModuleIO bottomLeftShooterModuleIO,
            ShooterModuleIO bottomRightShooterModuleIO) {
        shooterModules = new ShooterModule[] {
            new ShooterModule(topRightShooterModuleIO, "TopRight"),
            new ShooterModule(topLeftShooterModuleIO, "TopLeft"),
            new ShooterModule(bottomLeftShooterModuleIO, "BottomLeft"),
            new ShooterModule(bottomRightShooterModuleIO, "BottomRight")
        };

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        edu.wpi.first.units.Units.Volts.of(10),
                        edu.wpi.first.units.Units.Seconds.of(13),
                        (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> setVoltage(voltage.in(edu.wpi.first.units.Units.Volts), 0), null, this));
    }

    @Override
    public void periodic() {
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    for (int i = 0; i < 4; i++) {
                        shooterModules[i].configurePID(kP.get(), kI.get(), kD.get());
                    }
                },
                kP,
                kI,
                kD);

        for (var shooterModule : shooterModules) {
            shooterModule.periodic();
        }
    }

    protected void setVoltage(double volts, int... shooterModules) {
        currentGoal = Goal.IDLE;
        for (var shooterModule : shooterModules) {
            this.shooterModules[shooterModule].setVoltage(volts);
        }
    }

    protected void setVelocity(double RPM, int... shooterModules) {
        currentGoal = Goal.IDLE;
        for (var shooterModule : shooterModules) {
            this.shooterModules[shooterModule].setVelocity(RPM);
        }
    }

    protected void setGoal(Goal goal) {
        currentGoal = goal;
        for (int i = 0; i < 4; i++) {
            shooterModules[i].setVelocity(goal.getSpeeds()[i]);
        }
    }

    protected double getShooterVelocityRPM(int shooterModule) {
        return shooterModules[shooterModule].getVelocityRPM();
    }

    protected void stopShooter() {
        currentGoal = Goal.IDLE;
        for (var shooterModule : shooterModules) {
            shooterModule.stopShooter();
        }
    }

    protected void zeroEncoder() {
        for (var shooterModule : shooterModules) {
            shooterModule.zeroEncoder();
        }
    }
}
