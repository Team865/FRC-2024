package ca.warp7.frc2024.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkMAX550 implements ShooterModuleIO {
    private CANSparkMax shooterSparkMAX;
    private RelativeEncoder shooterInternalEncoder;

    public ShooterIOSparkMAX550(int shooterSparkMAXID) {
        shooterSparkMAX = new CANSparkMax(shooterSparkMAXID, MotorType.kBrushless);

        // TODO: Might want to implement 6328 type SparkMAX manager
        shooterSparkMAX.restoreFactoryDefaults();
        shooterSparkMAX.setCANTimeout(250);
        shooterSparkMAX.enableVoltageCompensation(12.0);
        shooterSparkMAX.setSmartCurrentLimit(20);
        shooterSparkMAX.setIdleMode(IdleMode.kCoast);
        shooterSparkMAX.burnFlash();

        shooterInternalEncoder = shooterSparkMAX.getEncoder();
    }

    @Override
    public void updateInputs(ShooterModuleIOInputs inputs) {
        inputs.shooterVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(shooterInternalEncoder.getVelocity());
        inputs.shooterAppliedVolts = shooterSparkMAX.getAppliedOutput();
        inputs.shooterCurrentAmps = shooterSparkMAX.getOutputCurrent();
    }
}
