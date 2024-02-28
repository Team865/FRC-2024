package ca.warp7.frc2024.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ShooterModuleIOSparkMax550 implements ShooterModuleIO {
    private final CANSparkMax shooterSparkMax;
    private final RelativeEncoder shooterInternalEncoder;

    private final SparkPIDController internalFeedback;

    public ShooterModuleIOSparkMax550(int shooterSparkMaxId, boolean invert) {
        shooterSparkMax = new CANSparkMax(shooterSparkMaxId, MotorType.kBrushless);

        // TODO: Might want to implement some type of SparkMax manager:
        // https://www.chiefdelphi.com/t/revlib-2024-burnflash-spark-max-unreliabilty/446192?u=dangosaurus
        shooterSparkMax.restoreFactoryDefaults();
        shooterSparkMax.setCANTimeout(250);
        shooterSparkMax.enableVoltageCompensation(12.0);
        shooterSparkMax.setSmartCurrentLimit(15);
        shooterSparkMax.setIdleMode(IdleMode.kCoast);
        if (invert) {
            shooterSparkMax.setInverted(true);
        }
        shooterSparkMax.burnFlash();

        // Specify encoder type: https://www.chiefdelphi.com/t/psa-new-crash-bug-in-revlib-2024-2-2/456242
        shooterInternalEncoder = shooterSparkMax.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        internalFeedback = shooterSparkMax.getPIDController();
    }

    @Override
    public void updateInputs(ShooterModuleIOInputs inputs) {
        inputs.shooterPositionRad = Units.rotationsToRadians(shooterInternalEncoder.getPosition());
        inputs.shooterVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(shooterInternalEncoder.getVelocity());
        inputs.shooterAppliedVolts = shooterSparkMax.getAppliedOutput() * shooterSparkMax.getBusVoltage();
        inputs.shooterCurrentAmps = shooterSparkMax.getOutputCurrent();
    }

    // TODO: Using hardcoded values instead of arguments for tuning
    @Override
    public void configureShooterPID(double kP, double kI, double kD) {
        internalFeedback.setP(0, 0);
        internalFeedback.setI(0, 0);
        internalFeedback.setD(0, 0);
        internalFeedback.setFF(0.0001, 0);
    }

    @Override
    public void setShooterVelocity(double velocityRadPerSec, double arbFfVolts) {
        // TODO: Revert to using offboard feedforward after fixing sporadic SparkMax behavior
        internalFeedback.setReference(
                Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec), ControlType.kVelocity, 0);
    }

    @Override
    public void setShooterVoltage(double volts) {
        shooterSparkMax.setVoltage(volts);
    }

    @Override
    public void stopShooter() {
        shooterSparkMax.stopMotor();
    }

    @Override
    public void zeroEncoder() {
        shooterInternalEncoder.setPosition(0);
    }
}
