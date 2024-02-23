package ca.warp7.frc2024.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
    private DCMotorSim topRightSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.001);
    private double topRightVoltsApplied = 0.0;

    private DCMotorSim bottomRightSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.001);
    private double bottomRightVoltsApplied = 0.0;

    private DCMotorSim topLeftSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.001);
    private double topLeftVoltsApplied = 0.0;

    private DCMotorSim bottomLeftSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.001);
    private double bottomLeftVoltsApplied = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        this.topRightSim.update(0.02);
        this.bottomRightSim.update(0.02);

        this.topLeftSim.update(0.02);
        this.bottomLeftSim.update(0.02);

        inputs.topRight.VelocityRPM = this.topRightSim.getAngularVelocityRPM();
        inputs.topRight.CurrentDraw = this.topRightSim.getCurrentDrawAmps();
        inputs.topRight.VoltsApplied = this.topRightVoltsApplied;

        inputs.bottomRight.VelocityRPM = this.bottomRightSim.getAngularVelocityRPM();
        inputs.bottomRight.CurrentDraw = this.bottomRightSim.getCurrentDrawAmps();
        inputs.bottomRight.VoltsApplied = this.bottomRightVoltsApplied;

        inputs.topLeft.VelocityRPM = this.topLeftSim.getAngularVelocityRPM();
        inputs.topLeft.CurrentDraw = this.topLeftSim.getCurrentDrawAmps();
        inputs.topLeft.VoltsApplied = this.topLeftVoltsApplied;

        inputs.bottomLeft.VelocityRPM = this.bottomLeftSim.getAngularVelocityRPM();
        inputs.bottomLeft.CurrentDraw = this.bottomLeftSim.getCurrentDrawAmps();
        inputs.bottomLeft.VoltsApplied = this.bottomLeftVoltsApplied;
    }

    @Override
    public void setTopRightVoltage(double volts) {
        this.topRightVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.topRightSim.setInputVoltage(this.topRightVoltsApplied);
    }

    @Override
    public void setTopLeftVoltage(double volts) {
        this.topLeftVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.topLeftSim.setInputVoltage(this.topLeftVoltsApplied);
    }

    @Override
    public void setBottomRightVoltage(double volts) {
        this.bottomRightVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.bottomRightSim.setInputVoltage(this.bottomRightVoltsApplied);
    }

    @Override
    public void setBottomLeftVoltage(double volts) {
        this.bottomLeftVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.bottomLeftSim.setInputVoltage(this.bottomLeftVoltsApplied);
    }
}
