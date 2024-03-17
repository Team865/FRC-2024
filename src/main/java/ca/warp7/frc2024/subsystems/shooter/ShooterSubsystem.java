package ca.warp7.frc2024.subsystems.shooter;

import ca.warp7.frc2024.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterModule[] shooterModules;

    private final SysIdRoutine sysId;

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Gains/kP", 0.00065);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/Gains/kI", 0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Gains/kD", 0.01);

    public ShooterSubsystem(
            ShooterModuleIO topRightShooterModuleIO,
            ShooterModuleIO topLeftShooterModuleIO,
            ShooterModuleIO bottomLeftShooterModuleIO,
            ShooterModuleIO bottomRightShooterModuleIO) {
        shooterModules = new ShooterModule[] {
            new ShooterModule(topRightShooterModuleIO, 0, "TopRight"),
            new ShooterModule(topLeftShooterModuleIO, 1, "TopLeft"),
            new ShooterModule(bottomLeftShooterModuleIO, 2, "BottomLeft"),
            new ShooterModule(bottomRightShooterModuleIO, 3, "BottomRight")
        };

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> setVoltage(voltage.in(edu.wpi.first.units.Units.Volts), 0), null, this));
    }

    @Override
    public void periodic() {
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    shooterModules[0].configurePID(kP.get(), kI.get(), kD.get());
                    shooterModules[1].configurePID(kP.get(), kI.get(), kD.get());
                    shooterModules[2].configurePID(kP.get(), kI.get(), kD.get());
                    shooterModules[3].configurePID(kP.get(), kI.get(), kD.get());
                },
                kP,
                kI,
                kD);

        for (var shooterModule : shooterModules) {
            shooterModule.periodic();
        }
    }

    private void setVoltage(double volts, int... shooterModules) {
        for (var shooterModule : shooterModules) {
            this.shooterModules[shooterModule].setVoltage(volts);
        }
    }

    public void setVelocity(double RPM, int... shooterModules) {
        for (var shooterModule : shooterModules) {
            this.shooterModules[shooterModule].setVelocity(RPM);
        }
    }

    public double getShooterVelocityRPM(int shooterModule) {
        return shooterModules[shooterModule].getVelocityRPM();
    }

    public void stopShooter() {
        for (var shooterModule : shooterModules) {
            shooterModule.stopShooter();
        }
    }

    private void zeroEncoder() {
        for (var shooterModule : shooterModules) {
            shooterModule.zeroEncoder();
        }
    }

    public Command runVelocityCommand(double RPM, int... shooterModules) {
        return this.runOnce(() -> setVelocity(RPM, shooterModules));
    }

    public Command runVelocityCommandEnds(double RPM, int... shooterModules) {
        return this.startEnd(() -> setVelocity(RPM, shooterModules), () -> stopShooter());
    }

    public Command runVoltageCommand(double volts, int... shooterModules) {
        return this.runOnce(() -> setVoltage(volts, shooterModules));
    }

    public Command runVoltageCommandEnds(double volts, int... shooterModules) {
        return this.startEnd(() -> setVoltage(volts, shooterModules), () -> stopShooter());
    }

    public Command stopShooterCommand() {
        return this.runOnce(() -> stopShooter());
    }

    public Command zeroEncoderCommand() {
        return this.runOnce(() -> zeroEncoder());
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
