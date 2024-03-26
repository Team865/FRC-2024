package ca.warp7.frc2024.subsystems.arm;

import static ca.warp7.frc2024.Constants.CURRENT_MODE;

import ca.warp7.frc2024.util.LoggedTunableNumber;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.apache.commons.math3.fitting.WeightedObservedPoint;

public final class ArmConstants {
    public static final record Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

    public static final Gains GAINS =
            switch (CURRENT_MODE) {
                case REAL -> new Gains(0.33, 0.02, 0.003, 0, 1.27, 0.02, 0.52);
                case SIM -> new Gains(1, 0, 0, 0, 0, 0, 0);
                default -> new Gains(0, 0, 0, 0, 0, 0, 0);
            };

    public static final double MAX_VELOCITY_DEG = 4000;
    public static final double MAX_ACCELERATION_DEG = 2000;

    @RequiredArgsConstructor
    public static enum Goal {
        HANDOFF_INTAKE(new LoggedTunableNumber("Arm/Goals/HandoffIntakeDegrees", 0)),
        STATION_INTAKE(new LoggedTunableNumber("Arm/Goals/StationIntakeDegrees", 0)),
        AMP(new LoggedTunableNumber("Arm/Goals/AmpDegrees", 68)),
        TRAP(new LoggedTunableNumber("Arm/Goals/TrapDegrees", 3)),
        PODIUM(new LoggedTunableNumber("Arm/Goals/PodiumDegrees", 63)),
        SUBWOOFER(new LoggedTunableNumber("Arm/Goals/SubwooferDegrees", 50)),
        BLOCKER(new LoggedTunableNumber("Arm/Goals/BlockerDegrees", 80)),
        INTERPOLATION(() -> 0),
        IDLE(() -> 0);

        private final DoubleSupplier armGoalSupplier;

        public double getDegrees() {
            return armGoalSupplier.getAsDouble();
        }

        public double getRadians() {
            return Units.degreesToRadians(getDegrees());
        }
    }

    public static final List<WeightedObservedPoint> POINTS = Arrays.asList(
            new WeightedObservedPoint(1, 1.4, 48.0),
            new WeightedObservedPoint(1, 2.0, 56.5),
            new WeightedObservedPoint(1, 2.5, 61.0),
            new WeightedObservedPoint(1, 3.0, 64.0),
            new WeightedObservedPoint(1, 3.5, 66.0),
            new WeightedObservedPoint(1, 4.0, 67.0),
            new WeightedObservedPoint(1, 4.5, 68.0),
            new WeightedObservedPoint(1, 5.15, 69.0),
            new WeightedObservedPoint(1, 5.47, 69.5),
            new WeightedObservedPoint(1, 6, 70));

    // angle in degrees,
    public static final double[] ANGLE = {50.0, 63.0, 66};

    // distance in meters
    public static final double[] DISTANCE = {1.4, 3.17, 3.79};
}
