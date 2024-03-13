package ca.warp7.frc2024.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/* IO implementation for NavX */
public class GyroIONavX implements GyroIO {
    private final AHRS navx = new AHRS(SPI.Port.kMXP);

    public GyroIONavX() {}

    public void updateInputs(GyroIOInputs inputs) {
        inputs.gyroConnected = navx.isConnected();
        inputs.gyroYaw = navx.getRotation2d();
    }

    @Override
    public void zeroYaw() {
        navx.zeroYaw();
    }
}
