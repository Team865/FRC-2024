package ca.warp7.frc2024.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    private static final double ARM_LENGTH = Units.inchesToMeters(20);
    private static final double ARM_MASS = Units.lbsToKilograms(16.5);

    private double armAppliedVolts = 0.0;

    private final Rotation2d randomInitialPosition =
            Rotation2d.fromDegrees(Math.random() * 80); // Initialize to random value

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getNEO(2),
            20 * (74 / 22),
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(ARM_LENGTH), Units.lbsToKilograms(ARM_MASS)),
            ARM_LENGTH,
            Units.degreesToRadians(120),
            Units.degreesToRadians(200),
            false,
            Units.degreesToRadians(120 + randomInitialPosition.getDegrees()),
            VecBuilder.fill(2 * Math.PI / 4096));

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if (DriverStation.isDisabled()) {}

        armSim.update(LOOP_PERIOD_SECS);

        inputs.armInternalIncrementalPosition = Rotation2d.fromRadians(armSim.getAngleRads());
        inputs.armExternalIncrementalPosition =
                Rotation2d.fromRadians(armSim.getAngleRads()).minus(Rotation2d.fromDegrees(120));
        inputs.armExternalAbsolutePosition = Rotation2d.fromRadians(armSim.getAngleRads());
        inputs.armInternalVelocityRadPerSec = armSim.getVelocityRadPerSec();
        inputs.armAppliedVolts = new double[] {armAppliedVolts};
        inputs.armCurrentAmps = new double[] {armSim.getCurrentDrawAmps()};
    }

    @Override
    public void setVoltage(double volts) {
        armAppliedVolts = MathUtil.clamp(volts, -12, 12);
        armSim.setInputVoltage(armAppliedVolts);
    }
}
