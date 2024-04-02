package ca.warp7.frc2024.subsystems.drivetrain;

import static ca.warp7.frc2024.Constants.CURRENT_MODE;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import lombok.RequiredArgsConstructor;

public final class DrivetrainConstants {
    public static final double DRIVE_BASE_X = Units.inchesToMeters(24.750);
    public static final double DRIVE_BASE_Y = Units.inchesToMeters(24.750);

    public static final double DRIVE_BASE_RADIUS = Math.hypot(DRIVE_BASE_X / 2.0, DRIVE_BASE_Y / 2.0);

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.9);
    public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;

    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(16.5);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final Translation2d[] SWERVE_MODULE_TRANSLATIONS = new Translation2d[] {
        new Translation2d(DRIVE_BASE_X / 2.0, -DRIVE_BASE_Y / 2.0),
        new Translation2d(DRIVE_BASE_X / 2.0, DRIVE_BASE_Y / 2.0),
        new Translation2d(-DRIVE_BASE_X / 2.0, DRIVE_BASE_Y / 2.0),
        new Translation2d(-DRIVE_BASE_X / 2.0, -DRIVE_BASE_Y / 2.0)
    };

    @RequiredArgsConstructor
    public enum HeadingSnapPoint {
        NONE(0, 0),
        HOLD(0, 0),
        AMP(90, 90),
        FEEDER(-45, -135),
        PASSING(-45, -135);

        private final double heading_blue;
        private final double heading_red;

        public double getAllianceHeading() {
            if (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red) {
                return heading_red;
            } else {
                return heading_blue;
            }
        }
    }

    public static final record Gains(double kP, double kI, double kD, double kS, double kV) {}

    public static final Gains DRIVE_GAINS =
            switch (CURRENT_MODE) {
                    // case REAL -> new Gains(0.1, 0.0, 0.0, 0.23466, 0.12025);
                case REAL -> new Gains(0.15226, 0.0, 0.0, 0.091932, 0.11715);
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
