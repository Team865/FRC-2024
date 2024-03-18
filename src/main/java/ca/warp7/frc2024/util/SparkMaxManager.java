package ca.warp7.frc2024.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

// Implementation based on work by FRC-3481, available at:
// https://github.com/BroncBotz3481/YAGSL/blob/main/swervelib/motors/SparkMaxSwerve.java
public class SparkMaxManager {
    private static final int MAX_RETRIES = 5;
    private static final double BURN_SLEEP = 0.2;

    private static REVLibError state;

    /** Repeatedly tries to apply configuration to SparkMax up to max retries
     *
     * @param motor CANSparkMax object
     * @param config Lambda of configuration that returns a REVLibError state
     */
    public static void safeSparkMax(CANSparkMax motor, Supplier<REVLibError> config) {
        for (int i = 0; i < MAX_RETRIES; i++) {
            state = config.get();

            if (state == REVLibError.kOk) {
                return;
            }
        }
        DriverStation.reportWarning("Failure configuring SparkMax " + motor.getDeviceId() + ": " + state.name(), false);
    }

    public static void safeBurnSparkMax(CANSparkMax motor) {
        Timer.delay(BURN_SLEEP);
        safeSparkMax(motor, () -> motor.burnFlash());
        Timer.delay(BURN_SLEEP);
    }
}
