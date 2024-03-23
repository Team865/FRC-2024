package ca.warp7.frc2024;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import lombok.RequiredArgsConstructor;

public final class Constants {
    public static enum MODE {
        REAL,
        SIM,
        REPLAY
    }

    public static final MODE CURRENT_MODE = MODE.REAL;
    public static final boolean TUNING_MODE = true;

    public static final class CLIMBER {
        @RequiredArgsConstructor
        public static enum STATE {
            CLIMBER_START(0),
            CLIMBER_START_HIGHEST(450),
            CLIMBER_END_HIGHEST(750),
            CLIMBER_END(1250);

            private final double position;

            public double getStatePosition() {
                return position;
            }
        }
    }

    public static final class DRIVETRAIN {
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
    }

    public static final class OI {
        public static final double DEADBAND = 0.1;
    }

    public static final class kIntake {
        public static final double kIntakeSpeed = 0;
        public static final double kOutakeSpeed = 0;
        public static final int kIntakeNeoID = 0;
    }
}
