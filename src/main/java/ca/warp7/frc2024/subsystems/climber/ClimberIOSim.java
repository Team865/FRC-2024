package ca.warp7.frc2024.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberIOSim implements ClimberIO {

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberInternalPosition = new Rotation2d();
    }
}
