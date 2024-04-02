package ca.warp7.frc2024;

import static edu.wpi.first.units.Units.*;

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

    public static final class OI {
        public static final double DEADBAND = 0.1;
        public static final double TRIGGER_THRESHOLD = 0.1;
    }

    public static final class kIntake {
        public static final double kIntakeSpeed = 0;
        public static final double kOutakeSpeed = 0;
        public static final int kIntakeNeoID = 0;
    }
}
