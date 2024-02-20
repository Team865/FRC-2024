package ca.warp7.frc2024.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveModuleIOFalcon500 implements SwerveModuleIO {
    private final TalonFX driveTalonFX;
    private final TalonFX steerTalonFX;
    private final CANcoder cancoder;

    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;

    private final StatusSignal<Double> steerAbsolutePosition;
    private final StatusSignal<Double> steerPosition;
    private final StatusSignal<Double> steerVelocity;
    private final StatusSignal<Double> steerAppliedVolts;
    private final StatusSignal<Double> steerCurrent;

    private final Rotation2d absoluteEncoderOffset;

    public SwerveModuleIOFalcon500(
            int driveTalonID, int steerTalonID, int cancoderID, Rotation2d absoluteEncoderOffset) {
        driveTalonFX = new TalonFX(driveTalonID);
        steerTalonFX = new TalonFX(steerTalonID);
        cancoder = new CANcoder(cancoderID);
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveTalonFX.getConfigurator().apply(driveConfig);

        var steerConfig = new TalonFXConfiguration();
        steerConfig.CurrentLimits.StatorCurrentLimit = 30.0;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerTalonFX.getConfigurator().apply(steerConfig);

        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        this.drivePosition = driveTalonFX.getPosition();
        this.driveVelocity = driveTalonFX.getVelocity();
        this.driveAppliedVolts = driveTalonFX.getMotorVoltage();
        this.driveCurrent = driveTalonFX.getStatorCurrent();

        this.steerAbsolutePosition = cancoder.getAbsolutePosition();
        this.steerPosition = steerTalonFX.getPosition();
        this.steerVelocity = steerTalonFX.getVelocity();
        this.steerAppliedVolts = steerTalonFX.getMotorVoltage();
        this.steerCurrent = steerTalonFX.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(250, drivePosition, steerPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                steerAbsolutePosition,
                steerVelocity,
                steerAppliedVolts,
                steerCurrent);
        driveTalonFX.optimizeBusUtilization();
        steerTalonFX.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                steerAbsolutePosition,
                steerAbsolutePosition,
                steerAppliedVolts,
                steerCurrent);

        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / 15;
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / 15;
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

        inputs.steerAbsolutePosition = Rotation2d.fromRotations(steerAbsolutePosition.getValueAsDouble())
                .minus(absoluteEncoderOffset);
        inputs.steerPosition = Rotation2d.fromRotations(steerPosition.getValueAsDouble() / 15);
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocity.getValueAsDouble()) / 15;
        inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
        inputs.steerCurrentAmps = new double[] {steerCurrent.getValueAsDouble()};
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveTalonFX.setControl(new VoltageOut(volts));
    }

    @Override
    public void setSteerVoltage(double volts) {
        steerTalonFX.setControl(new VoltageOut(volts));
    }
}
