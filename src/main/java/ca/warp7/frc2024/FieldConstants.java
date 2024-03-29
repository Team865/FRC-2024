package ca.warp7.frc2024;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import lombok.RequiredArgsConstructor;

public final class FieldConstants {
    @RequiredArgsConstructor
    public enum PointOfInterest {
        NONE(new Translation2d()),
        SPEAKER(new Translation2d(0.36, 5.55)),
        SPEAKER_WALL(new Translation2d(0.0, 5.55)),
        AMP(new Translation2d(1.82, 8.2));

        private final Translation2d blueTranslation;

        public Translation2d getAllianceTranslation() {
            if (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red) {
                return new Translation2d(16.54 - blueTranslation.getX(), blueTranslation.getY());
            } else {
                return blueTranslation;
            }
        }
    }
}
