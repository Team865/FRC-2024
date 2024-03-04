package ca.warp7.frc2024.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

    private double intakeAppliedVolts = 0.0;
    private final double LOOP_PERIOD_SECS = 0.02;
    private final DCMotorSim intakeSim = new DCMotorSim(DCMotor.getNEO(1), 4, 0.0001);

    public void updateInputs(IntakeIOInputs inputs) {
        intakeSim.update(LOOP_PERIOD_SECS);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeAppliedVolts = MathUtil.clamp(volts, -12, 12);
        intakeSim.setInputVoltage(intakeAppliedVolts);
    }
}
