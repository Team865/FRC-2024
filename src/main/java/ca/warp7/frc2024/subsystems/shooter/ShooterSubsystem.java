package ca.warp7.frc2024.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterModule[] shooterModules;

    public ShooterSubsystem(
            ShooterModuleIO topRightShooterModuleIO,
            ShooterModuleIO topLeftShooterModuleIO,
            ShooterModuleIO bottomLeftShooterModuleIO,
            ShooterModuleIO bottomRightShooterModuleIO) {
        shooterModules = new ShooterModule[] {
            new ShooterModule(topRightShooterModuleIO, 0, "Top Right"),
            new ShooterModule(topLeftShooterModuleIO, 1, "Top Left"),
            new ShooterModule(bottomLeftShooterModuleIO, 2, "Bottom Left"),
            new ShooterModule(bottomRightShooterModuleIO, 3, "Bottom Right")
        };
    }

    public void setShooterRPM(double RPM, int... shooterModules) {
        for (var shooterModule : shooterModules) {
            try {
                this.shooterModules[shooterModule].setTargetRPM(RPM); // TODO: I think this makes sense but test in sim
            } catch (Exception e) {
                throw new ArrayIndexOutOfBoundsException("Invalid shooter module index");
            }
        }
    }

    @Override
    public void periodic() {
        for (var ShooterModule : shooterModules) {
            ShooterModule.periodic();
        }
    }
}
