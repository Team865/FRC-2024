package ca.warp7.frc2024.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
    private DCMotorSim topRightOutrunnerSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.0001);
    private double topRightOutrunnerVoltsApplied = 0.0;

    private DCMotorSim bottomRightOutrunnerSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.0001);
    private double bottomRightOutrunnerVoltsApplied = 0.0;

    private DCMotorSim topLeftOutrunnerSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.0001);
    private double topLeftOutrunnerVoltsApplied = 0.0;

    private DCMotorSim bottomLeftOutrunnerSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.0001);
    private double bottomLeftOutrunnerVoltsApplied = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        this.topRightOutrunnerSim.update(0.2);
        this.bottomRightOutrunnerSim.update(0.2);

        this.topLeftOutrunnerSim.update(0.2);
        this.bottomLeftOutrunnerSim.update(0.2);

        inputs.topRightOutrunner.VelocityRad = this.topRightOutrunnerSim.getAngularVelocityRadPerSec();
        inputs.topRightOutrunner.CurrentDraw = this.topRightOutrunnerSim.getCurrentDrawAmps();
        inputs.topRightOutrunner.VoltageApplied = this.topRightOutrunnerVoltsApplied;

        inputs.bottomRightOutrunner.VelocityRad = this.bottomRightOutrunnerSim.getAngularVelocityRadPerSec();
        inputs.bottomRightOutrunner.CurrentDraw = this.bottomRightOutrunnerSim.getCurrentDrawAmps();
        inputs.bottomRightOutrunner.VoltageApplied = this.bottomRightOutrunnerVoltsApplied;

        inputs.topLeftOutrunner.VelocityRad = this.topLeftOutrunnerSim.getAngularVelocityRadPerSec();
        inputs.topLeftOutrunner.CurrentDraw = this.topLeftOutrunnerSim.getCurrentDrawAmps();
        inputs.topLeftOutrunner.VoltageApplied = this.topLeftOutrunnerVoltsApplied;

        inputs.bottomLeftOutrunner.VelocityRad = this.bottomLeftOutrunnerSim.getAngularVelocityRadPerSec();
        inputs.bottomLeftOutrunner.CurrentDraw = this.bottomLeftOutrunnerSim.getCurrentDrawAmps();
        inputs.bottomLeftOutrunner.VoltageApplied = this.bottomLeftOutrunnerVoltsApplied;
    }

    @Override
    public void setTopRightVoltage(double volts) {
        this.topRightOutrunnerVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.topRightOutrunnerSim.setInputVoltage(this.topRightOutrunnerVoltsApplied);
    }

    @Override
    public void setTopLeftVoltage(double volts) {
        this.topLeftOutrunnerVoltsApplied = MathUtil.clamp(-volts, -12, 12);
        this.topLeftOutrunnerSim.setInputVoltage(this.topLeftOutrunnerVoltsApplied);
    }

    @Override
    public void setbottomRightVoltage(double volts) {
        this.bottomRightOutrunnerVoltsApplied = MathUtil.clamp(-volts, -12, 12);
        this.bottomRightOutrunnerSim.setInputVoltage(this.bottomRightOutrunnerVoltsApplied);
    }

    @Override
    public void setbottomLeftVoltage(double volts) {
        this.bottomLeftOutrunnerVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.bottomLeftOutrunnerSim.setInputVoltage(this.bottomLeftOutrunnerVoltsApplied);
    }
}
