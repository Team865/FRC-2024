package ca.warp7.frc2024.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim driveSim = new DCMotorSim(DCMotor.getFalcon500(1), 6.75, 0.025);
    private DCMotorSim steerSim = new DCMotorSim(DCMotor.getFalcon500(1), 150 / 7, 0.004);

    private final Rotation2d steerAbsoluteInitialPosition =
            new Rotation2d(Math.random() * 2.0 * Math.PI); // Initialize to random value
    private double driveAppliedVolts = 0.0;
    private double steerAppliedVolts = 0.0;

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveSim.update(LOOP_PERIOD_SECS);
        steerSim.update(LOOP_PERIOD_SECS);

        inputs.drivePositionRad = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();

        inputs.steerAbsolutePosition =
                new Rotation2d(steerSim.getAngularPositionRad()).plus(steerAbsoluteInitialPosition);
        inputs.steerPosition = new Rotation2d(steerSim.getAngularPositionRad());
        inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVolts = steerAppliedVolts;
        inputs.steerCurrentAmps = steerSim.getCurrentDrawAmps();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setSteerVoltage(double volts) {
        steerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        steerSim.setInputVoltage(steerAppliedVolts);
    }
}
