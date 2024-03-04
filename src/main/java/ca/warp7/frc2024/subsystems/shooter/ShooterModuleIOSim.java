package ca.warp7.frc2024.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// TODO: Sim currently broken
public class ShooterModuleIOSim implements ShooterModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim shooterSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.001);
    private double shooterAppliedVolts = 0.0;

    @Override
    public void updateInputs(ShooterModuleIOInputs inputs) {
        shooterSim.update(LOOP_PERIOD_SECS);

        inputs.shooterVelocityRadPerSec = shooterSim.getAngularVelocityRadPerSec();
        inputs.shooterAppliedVolts = shooterAppliedVolts;
        inputs.shooterCurrentAmps = shooterSim.getCurrentDrawAmps();
    }

    @Override
    public void runShooterVolts(double volts) {
        shooterAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        shooterSim.setInputVoltage(shooterAppliedVolts);
    }
}
