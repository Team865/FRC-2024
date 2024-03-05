package ca.warp7.frc2024;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static enum MODE {
        REAL,
        SIM,
        REPLAY
    }

    public static final MODE CURRENT_MODE = MODE.REAL;
    public static final boolean TUNING_MODE = true;

    public final class ARM {
        public record Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

        public static final ARM.Gains GAINS =
                switch (CURRENT_MODE) {
                    case REAL -> new ARM.Gains(0.33, 0.02, 0.003, 0, 0, 0, 0);
                    case SIM -> new ARM.Gains(1, 0, 0, 0, 0, 0, 0);
                    default -> new ARM.Gains(0, 0, 0, 0, 0, 0, 0);
                };

        public static final double MAX_VELOCITY_DEG = 4000;
        public static final double MAX_ACCELERATION_DEG = 2000;
    }

    public class DRIVETRAIN {
        public static final double DRIVE_BASE_X = Units.inchesToMeters(24.750);
        public static final double DRIVE_BASE_Y = Units.inchesToMeters(24.750);

        private static final double DRIVE_BASE_RADIUS = Math.hypot(DRIVE_BASE_X / 2.0, DRIVE_BASE_Y / 2.0);

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2;

        public static final double MAX_LINEAR_SPEED = Units.feetToMeters(16.5);
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

        public static final Translation2d[] SWERVE_MODULE_TRANSLATIONS = new Translation2d[] {
            new Translation2d(DRIVE_BASE_X / 2.0, -DRIVE_BASE_Y / 2.0),
            new Translation2d(DRIVE_BASE_X / 2.0, DRIVE_BASE_Y / 2.0),
            new Translation2d(-DRIVE_BASE_X / 2.0, DRIVE_BASE_Y / 2.0),
            new Translation2d(-DRIVE_BASE_X / 2.0, -DRIVE_BASE_Y / 2.0)
        };
    }

    public class OI {
        public static final double DEADBAND = 0.1;
    }

    public static final class kIntake {
        public static final double kIntakeSpeed = 0;
        public static final double kOutakeSpeed = 0;
        public static final int kIntakeNeoID = 0;
    }
}
