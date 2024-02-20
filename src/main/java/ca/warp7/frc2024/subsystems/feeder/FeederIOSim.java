package ca.warp7.frc2024.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {
    private DCMotorSim topFeederSim = new DCMotorSim(DCMotor.getNeo550(1), 5, 0.0001);
    private DCMotorSim bottomFeederSim = new DCMotorSim(DCMotor.getNeo550(1), 5, 0.0001);
    private double topFeederVoltsApplied = 0.0;
    private double bottomFeederVoltsApplied = 0.0;

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.topFeederRoller.VelocityRad = this.topFeederSim.getAngularVelocityRadPerSec();
        inputs.topFeederRoller.CurrentDraw = this.topFeederSim.getCurrentDrawAmps();
        inputs.topFeederRoller.VoltageApplied = this.topFeederVoltsApplied;

        inputs.bottomFeederRoller.VelocityRad = this.bottomFeederSim.getAngularVelocityRadPerSec();
        inputs.bottomFeederRoller.CurrentDraw = this.bottomFeederSim.getCurrentDrawAmps();
        inputs.bottomFeederRoller.VoltageApplied = this.bottomFeederVoltsApplied;
    }

    @Override
    public void setFeederRollersVoltage(double volts) {
        this.topFeederVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.topFeederSim.setInputVoltage(this.topFeederVoltsApplied);

        this.bottomFeederVoltsApplied = MathUtil.clamp(-volts, -12, 12);
        this.bottomFeederSim.setInputVoltage(this.bottomFeederVoltsApplied);
    }

    @Override
    public void periodic() {
        this.topFeederSim.update(0.2);
        this.bottomFeederSim.update(0.2);
    }
}
