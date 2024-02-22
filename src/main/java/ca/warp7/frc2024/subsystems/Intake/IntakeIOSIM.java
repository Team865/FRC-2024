package ca.warp7.frc2024.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSIM implements IntakeIO {
    private double intakeAppliedVolts = 0.0;
    private final DCMotorSim intakeMotorSim = new DCMotorSim(DCMotor.getNEO(1), 4, 0.0001);

    @Override
    public void updateInputs(intakeIOInputs inputs) {
        inputs.intakeAppliedVolts = intakeAppliedVolts;
        inputs.intakeCurrentAmps = new double[] {};
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeAppliedVolts = MathUtil.clamp(volts, -12, 12);
        intakeMotorSim.setInputVoltage(intakeAppliedVolts);
    }
}
