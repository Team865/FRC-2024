package ca.warp7.frc2024.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

    private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private final double STEER_GEAR_RATIO = 150.0 / 7.0;

    private final double PHOENIX_TIMEOUT = 2.0;

    private final VoltageOut driveVoltageOut = new VoltageOut(0, false, false, false, false);
    private final VoltageOut steerVoltageOut = new VoltageOut(0, false, false, false, false);

    public SwerveModuleIOFalcon500(
            int driveTalonID, int steerTalonID, int cancoderID, Rotation2d absoluteEncoderOffset) {
        // TODO: Switch from waiting to using .waitForUpdate()
        // Timer.delay(1);
        driveTalonFX = new TalonFX(driveTalonID, "CANivore");

        // Timer.delay(1);
        steerTalonFX = new TalonFX(steerTalonID, "CANivore");

        // Timer.delay(1);
        cancoder = new CANcoder(cancoderID, "CANivore");

        this.absoluteEncoderOffset = absoluteEncoderOffset;

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveTalonFX.getConfigurator().apply(driveConfig);
        driveTalonFX.setPosition(0);

        var steerConfig = new TalonFXConfiguration();
        steerConfig.CurrentLimits.StatorCurrentLimit = 30.0;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerTalonFX.getConfigurator().apply(steerConfig);
        steerTalonFX.setPosition(0);

        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        this.drivePosition = driveTalonFX.getPosition().waitForUpdate(PHOENIX_TIMEOUT);
        this.driveVelocity = driveTalonFX.getVelocity().waitForUpdate(PHOENIX_TIMEOUT);
        this.driveAppliedVolts = driveTalonFX.getMotorVoltage().waitForUpdate(PHOENIX_TIMEOUT);
        this.driveCurrent = driveTalonFX.getStatorCurrent().waitForUpdate(PHOENIX_TIMEOUT);

        this.steerPosition = steerTalonFX.getPosition().waitForUpdate(PHOENIX_TIMEOUT);
        this.steerVelocity = steerTalonFX.getVelocity().waitForUpdate(PHOENIX_TIMEOUT);
        this.steerAppliedVolts = steerTalonFX.getMotorVoltage().waitForUpdate(PHOENIX_TIMEOUT);
        this.steerCurrent = steerTalonFX.getStatorCurrent().waitForUpdate(PHOENIX_TIMEOUT);

        this.steerAbsolutePosition = cancoder.getAbsolutePosition().waitForUpdate(PHOENIX_TIMEOUT);

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
        cancoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                steerPosition,
                steerVelocity,
                steerAbsolutePosition,
                steerAppliedVolts,
                steerCurrent);

        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

        inputs.steerAbsolutePosition = Rotation2d.fromRotations(steerAbsolutePosition.getValueAsDouble())
                .minus(absoluteEncoderOffset);
        inputs.steerPosition = Rotation2d.fromRotations(steerPosition.getValueAsDouble() / STEER_GEAR_RATIO);
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocity.getValueAsDouble()) / STEER_GEAR_RATIO;
        inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
        inputs.steerCurrentAmps = steerCurrent.getValueAsDouble();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveTalonFX.setControl(driveVoltageOut.withOutput(volts));
    }

    @Override
    public void setSteerVoltage(double volts) {
        steerTalonFX.setControl(steerVoltageOut.withOutput(volts));
    }
}
