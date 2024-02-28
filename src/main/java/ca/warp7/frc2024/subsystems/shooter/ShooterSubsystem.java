package ca.warp7.frc2024.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterModule[] shooterModules;

    private final SysIdRoutine sysId;

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
                        (voltage) -> setShooterVolts(voltage.in(edu.wpi.first.units.Units.Volts), 0), null, this));
    }

    public void setShooterVolts(double volts, int shooterModule) {
        this.shooterModules[shooterModule].setVolts(volts);

        // TODO: Temporarily don't use varargs
        // for (var shooterModule : shooterModules) {
        //     this.shooterModules[shooterModule].setVolts(volts);
        // }
    }

    public void setShooterRPM(double RPM, int shooterModule) {
        this.shooterModules[shooterModule].setTargetVelocity(RPM);

        // TODO: Temporarily don't use varargs
        // for (var shooterModule : shooterModules) {
        //     this.shooterModules[shooterModule].setTargetVelocity(RPM);
        // }
    }

    public double getShooterVelocityRPM(int shooterModule) {
        return shooterModules[shooterModule].getVelocityRPM();
    }

    public void stopShooter() {
        for (var shooterModule : shooterModules) {
            shooterModule.stopShooter();
        }
    }

    public void zeroEncoder() {
        for (var shooterModule : shooterModules) {
            shooterModule.zeroEncoder();
        }
    }

    @Override
    public void periodic() {
        for (var shooterModule : shooterModules) {
            shooterModule.periodic();
        }
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
