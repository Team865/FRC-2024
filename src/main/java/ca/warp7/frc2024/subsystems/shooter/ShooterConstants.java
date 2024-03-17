package ca.warp7.frc2024.subsystems.shooter;

import static ca.warp7.frc2024.Constants.CURRENT_MODE;

import ca.warp7.frc2024.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public final class ShooterConstants {
    public static final record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

    public static final Gains Gains =
            switch (CURRENT_MODE) {
                case REAL -> new Gains(0.00065, 0.0, 0.1, 0.021949, 0.010201, 0.0010146);
                case SIM -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            };

    private static final record Speeds(
            LoggedTunableNumber topRight,
            LoggedTunableNumber topLeft,
            LoggedTunableNumber bottomLeft,
            LoggedTunableNumber bottomRight) {}

    private static final Speeds IDLE_SPEEDS = new Speeds(
            new LoggedTunableNumber("Shooter/Goals/Idle/TopRight", 0),
            new LoggedTunableNumber("Shooter/Goals/Idle/TopLeft", 0),
            new LoggedTunableNumber("Shooter/Goals/Idle/BottomLeft", 0),
            new LoggedTunableNumber("Shooter/Goals/Idle/BottomRight", 0));

    private static final Speeds TUNING_SPEEDS = new Speeds(
            new LoggedTunableNumber("Shooter/Goals/Tuning/TopRight", 0),
            new LoggedTunableNumber("Shooter/Goals/Tuning/TopLeft", 0),
            new LoggedTunableNumber("Shooter/Goals/Tuning/BottomLeft", 0),
            new LoggedTunableNumber("Shooter/Goals/Tuning/BottomRight", 0));

    private static final Speeds PODIUM_SPEEDS = new Speeds(
            new LoggedTunableNumber("Shooter/Goals/Podium/TopRight", 0),
            new LoggedTunableNumber("Shooter/Goals/Podium/TopLeft", 0),
            new LoggedTunableNumber("Shooter/Goals/Podium/BottomLeft", 0),
            new LoggedTunableNumber("Shooter/Goals/Podium/BottomRight", 0));

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(IDLE_SPEEDS),
        TUNING(TUNING_SPEEDS),
        PODIUM(PODIUM_SPEEDS);

        private final Speeds speeds;

        public double[] getSpeeds() {
            return new double[] {
                speeds.topRight.get(), speeds.topLeft.get(), speeds.bottomLeft.get(), speeds.bottomRight.get()
            };
        }
    }
}
