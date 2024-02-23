package ca.warp7.frc2024.subsystems.feeder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;

public class FeederIOSparkmax implements FeederIO {
    private CANSparkMax topRollerSparkMax;
    private RelativeEncoder topRollerEncoder;
    private double topRollerVoltsApplied;

    private CANSparkMax bottomRollerSparkMax;
    private RelativeEncoder bottomRollerEncoder;
    private double bottomRollerVoltsApplied;

    public FeederIOSparkmax(int topID, int bottomID) {
        topRollerSparkMax = new CANSparkMax(topID, MotorType.kBrushed);
        bottomRollerSparkMax = new CANSparkMax(bottomID, MotorType.kBrushless);

        setupMotorControllers();

        topRollerEncoder = topRollerSparkMax.getEncoder();
        bottomRollerEncoder = bottomRollerSparkMax.getEncoder();

        zeroEncoders();
    }

    private void setupMotorControllers() {
        topRollerSparkMax.restoreFactoryDefaults();
        bottomRollerSparkMax.restoreFactoryDefaults();

        topRollerSparkMax.setSmartCurrentLimit(20);
        bottomRollerSparkMax.setSmartCurrentLimit(20);

        topRollerSparkMax.setCANTimeout(0);
        bottomRollerSparkMax.setCANTimeout(0);

        topRollerSparkMax.setIdleMode(IdleMode.kBrake);
        bottomRollerSparkMax.setIdleMode(IdleMode.kBrake);

        bottomRollerSparkMax.setInverted(true);
    }

    private void zeroEncoders() {
        topRollerEncoder.setPosition(0);
        bottomRollerEncoder.setPosition(0);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.topRoller.VelocityRPM = this.topRollerEncoder.getVelocity();
        inputs.topRoller.CurrentDraw = this.topRollerSparkMax.getOutputCurrent();
        inputs.topRoller.VoltsApplied = this.topRollerVoltsApplied;

        inputs.bottomRoller.VelocityRPM = this.bottomRollerEncoder.getVelocity();
        inputs.bottomRoller.CurrentDraw = this.bottomRollerSparkMax.getOutputCurrent();
        inputs.bottomRoller.VoltsApplied = this.bottomRollerVoltsApplied;
    }

    @Override
    public void setTopVoltage(double volts) {
        this.topRollerVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.topRollerSparkMax.setVoltage(this.topRollerVoltsApplied);
    }

    @Override
    public void setBottomVoltage(double volts) {
        this.bottomRollerVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.bottomRollerSparkMax.setVoltage(this.bottomRollerVoltsApplied);
    }
}
