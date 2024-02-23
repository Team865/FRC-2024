package ca.warp7.frc2024.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {
    private DCMotorSim topRollerSim = new DCMotorSim(DCMotor.getNeo550(1), 5, 0.0001);
    private DCMotorSim bottomRollerSim = new DCMotorSim(DCMotor.getNeo550(1), 5, 0.0001);
    private double topRollerVoltsApplied = 0.0;
    private double bottomRollerVoltsApplied = 0.0;

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        this.topRollerSim.update(0.2);
        this.bottomRollerSim.update(0.2);

        inputs.topRoller.VelocityRPM = this.topRollerSim.getAngularVelocityRadPerSec();
        inputs.topRoller.CurrentDraw = this.topRollerSim.getCurrentDrawAmps();
        inputs.topRoller.VoltsApplied = this.topRollerVoltsApplied;

        inputs.bottomRoller.VelocityRPM = this.bottomRollerSim.getAngularVelocityRadPerSec();
        inputs.bottomRoller.CurrentDraw = this.bottomRollerSim.getCurrentDrawAmps();
        inputs.bottomRoller.VoltsApplied = this.bottomRollerVoltsApplied;
    }

    @Override
    public void setTopVoltage(double volts) {
        this.topRollerVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.topRollerSim.setInputVoltage(this.topRollerVoltsApplied);
    }

    @Override
    public void setBottomVoltage(double volts) {
        this.bottomRollerVoltsApplied = MathUtil.clamp(-volts, -12, 12);
        this.bottomRollerSim.setInputVoltage(this.bottomRollerVoltsApplied);
    }
}
