package ca.warp7.frc2024.subsystems.drivetrain;

import static ca.warp7.frc2024.Constants.CURRENT_MODE;

public final class DrivetrainConstants {
    public static final record Gains(double kP, double kI, double kD, double kS, double kV) {}

    public static final Gains DRIVE_GAINS =
            switch (CURRENT_MODE) {
                case REAL -> new Gains(0.1, 0.0, 0.0, 0.23466, 0.12025);
                case SIM -> new Gains(0.1, 0.0, 0.0, 0.0, 0.13);
                default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0);
            };

    public static final Gains STEER_GAINS =
            switch (CURRENT_MODE) {
                case REAL -> new Gains(6.5, 0.0, 0.0, 0.0, 0.0);
                case SIM -> new Gains(10, 0.0, 0.0, 0.0, 0.0);
                default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0);
            };
}
