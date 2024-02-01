package ca.warp7.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
    private DCMotorSim topFeederSim = new DCMotorSim(DCMotor.getNeo550(1), 5, 0.0001);
    private double topFeederVoltsApplied = 0.0;
    private double bottomFeederVoltsApplied = 0.0;

    private DCMotorSim bottomFeederSim = new DCMotorSim(DCMotor.getNeo550(1), 5, 0.0001);

    // Moi center = 0.0008505 J
    // for:
    // 0.453592 kg and 15cm
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
        inputs.topFeederRoller.VelocityRad = this.topFeederSim.getAngularVelocityRadPerSec();
        inputs.topFeederRoller.CurrentDraw = this.topFeederSim.getCurrentDrawAmps();
        inputs.topFeederRoller.VoltageApplied = this.topFeederVoltsApplied;

        inputs.bottomFeederRoller.VelocityRad = this.bottomFeederSim.getAngularVelocityRadPerSec();
        inputs.bottomFeederRoller.CurrentDraw = this.bottomFeederSim.getCurrentDrawAmps();
        inputs.bottomFeederRoller.VoltageApplied = this.bottomFeederVoltsApplied;

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
    public void setFeederRollersVoltage(double volts) {
        this.topFeederVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.topFeederSim.setInputVoltage(this.topFeederVoltsApplied);

        this.bottomFeederVoltsApplied = MathUtil.clamp(-volts, -12, 12);
        this.bottomFeederSim.setInputVoltage(this.bottomFeederVoltsApplied);
    }

    @Override
    public void setOutrunnersVoltage(double volts) {
        this.topRightOutrunnerVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.topRightOutrunnerSim.setInputVoltage(volts);

        this.bottomRightOutrunnerVoltsApplied = MathUtil.clamp(-volts, -12, 12);
        this.bottomRightOutrunnerSim.setInputVoltage(this.bottomRightOutrunnerVoltsApplied);

        this.topLeftOutrunnerVoltsApplied = MathUtil.clamp(-volts, -12, 12);
        this.topLeftOutrunnerSim.setInputVoltage(this.topLeftOutrunnerVoltsApplied);

        this.bottomLeftOutrunnerVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.bottomLeftOutrunnerSim.setInputVoltage(this.bottomLeftOutrunnerVoltsApplied);
    }

    @Override
    public void periodic() {
        this.topFeederSim.update(0.2);
        this.bottomFeederSim.update(0.2);

        this.topRightOutrunnerSim.update(0.2);
        this.bottomRightOutrunnerSim.update(0.2);

        this.topLeftOutrunnerSim.update(0.2);
        this.bottomLeftOutrunnerSim.update(0.2);
    }
}
