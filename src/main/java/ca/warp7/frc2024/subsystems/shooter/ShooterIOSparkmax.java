package ca.warp7.frc2024.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

public class ShooterIOSparkmax implements ShooterIO {
    private CANSparkMax topRightSparkMax;
    private CANSparkMax topLeftSparkMax;
    private CANSparkMax bottomRightSparkMax;
    private CANSparkMax bottomLeftSparkMax;

    private RelativeEncoder topRightEncoder;
    private RelativeEncoder topLeftEncoder;
    private RelativeEncoder bottomRightEncoder;
    private RelativeEncoder bottomLeftEncoder;

    private double topRightVoltsApplied = 0.0;
    private double bottomRightVoltsApplied = 0.0;
    private double topLeftVoltsApplied = 0.0;
    private double bottomLeftVoltsApplied = 0.0;

    public ShooterIOSparkmax(int topRightID, int topLeftID, int bottomRightID, int bottomLeftID) {
        topRightSparkMax = new CANSparkMax(topRightID, MotorType.kBrushless);
        topLeftSparkMax = new CANSparkMax(topLeftID, MotorType.kBrushless);
        bottomRightSparkMax = new CANSparkMax(bottomRightID, MotorType.kBrushless);
        bottomLeftSparkMax = new CANSparkMax(bottomLeftID, MotorType.kBrushless);

        setupMotorControllers();

        topRightEncoder = topRightSparkMax.getEncoder();
        topLeftEncoder = topLeftSparkMax.getEncoder();
        bottomRightEncoder = bottomRightSparkMax.getEncoder();
        bottomLeftEncoder = bottomLeftSparkMax.getEncoder();

        zeroEncoders();
    }

    private void setupMotorControllers() {
        topRightSparkMax.setSmartCurrentLimit(20);
        topLeftSparkMax.setSmartCurrentLimit(20);
        bottomRightSparkMax.setSmartCurrentLimit(20);
        bottomLeftSparkMax.setSmartCurrentLimit(20);

        topRightSparkMax.setCANTimeout(0);
        topLeftSparkMax.setCANTimeout(0);
        bottomRightSparkMax.setCANTimeout(0);
        bottomLeftSparkMax.setCANTimeout(0);

        topRightSparkMax.restoreFactoryDefaults();
        topLeftSparkMax.restoreFactoryDefaults();
        bottomRightSparkMax.restoreFactoryDefaults();
        bottomLeftSparkMax.restoreFactoryDefaults();

        topRightSparkMax.setIdleMode(IdleMode.kCoast);
        topLeftSparkMax.setIdleMode(IdleMode.kCoast);
        bottomRightSparkMax.setIdleMode(IdleMode.kCoast);
        bottomLeftSparkMax.setIdleMode(IdleMode.kCoast);
    }

    private void zeroEncoders() {
        topRightEncoder.setPosition(0);
        topLeftEncoder.setPosition(0);
        bottomRightEncoder.setPosition(0);
        bottomLeftEncoder.setPosition(0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topRight.VoltsApplied = this.topRightVoltsApplied;
        inputs.topRight.CurrentDraw = this.topRightSparkMax.getOutputCurrent();
        inputs.topRight.VelocityRPM = this.topRightEncoder.getVelocity();

        inputs.topLeft.VoltsApplied = this.topLeftVoltsApplied;
        inputs.topLeft.CurrentDraw = this.topLeftSparkMax.getOutputCurrent();
        inputs.topLeft.VelocityRPM = this.topLeftEncoder.getVelocity();

        inputs.bottomRight.VoltsApplied = this.bottomRightVoltsApplied;
        inputs.bottomRight.CurrentDraw = this.bottomRightSparkMax.getOutputCurrent();
        inputs.bottomRight.VelocityRPM = this.bottomRightEncoder.getVelocity();

        inputs.bottomLeft.VoltsApplied = this.bottomLeftVoltsApplied;
        inputs.bottomLeft.CurrentDraw = this.topRightSparkMax.getOutputCurrent();
        inputs.bottomLeft.VelocityRPM = this.bottomLeftEncoder.getVelocity();
    }

    @Override
    public void setTopRightVoltage(double volts) {
        this.topRightVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.topRightSparkMax.setVoltage(this.topRightVoltsApplied);
    }

    @Override
    public void setTopLeftVoltage(double volts) {
        this.topLeftVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.topLeftSparkMax.setVoltage(this.topLeftVoltsApplied);
    }

    @Override
    public void setBottomRightVoltage(double volts) {
        this.bottomRightVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.bottomRightSparkMax.setVoltage(this.bottomRightVoltsApplied);
    }

    @Override
    public void setBottomLeftVoltage(double volts) {
        this.bottomLeftVoltsApplied = MathUtil.clamp(volts, -12, 12);
        this.bottomLeftSparkMax.setVoltage(this.bottomLeftVoltsApplied);
    }
}
