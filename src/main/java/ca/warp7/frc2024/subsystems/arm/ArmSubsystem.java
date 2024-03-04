package ca.warp7.frc2024.subsystems.arm;

import static ca.warp7.frc2024.Constants.ARM.*;

import ca.warp7.frc2024.util.LoggedTunableNumber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
    /* AdvantageKit */
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    /* Controllers */
    private ProfiledPIDController feedback;
    private ArmFeedforward feedforward;

    /* Mechanism Visualization */
    private Mechanism2d mechanism;
    private MechanismRoot2d mechanismRoot;
    private MechanismLigament2d mechanismLigament;

    /* Gains */
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", GAINS.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", GAINS.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Gains/kD", GAINS.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/Gains/kD", GAINS.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/Gains/kD", GAINS.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/Gains/kD", GAINS.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/Gains/kD", GAINS.kG());
    private static final LoggedTunableNumber maxVelocity = new LoggedTunableNumber("Arm/Constraint", MAX_VELOCITY_DEG);
    private static final LoggedTunableNumber maxAcceleration =
            new LoggedTunableNumber("Arm/Constraint", MAX_ACCELERATION_DEG);

    /* Setpoints */
    @RequiredArgsConstructor
    public enum SETPOINT {
        HANDOFF_INTAKE(new LoggedTunableNumber("Arm/Setpoint/HandoffIntakeDegrees", 80)),
        STATION_INTAKE(new LoggedTunableNumber("Arm/Setpoint/StationIntakeDegrees", 80)),
        AMP(new LoggedTunableNumber("Arm/Setpoint/AmpDegrees", 60)),
        TRAP(new LoggedTunableNumber("Arm/Setpoint/TrapDegrees", 80)),
        IDLE(() -> 0);

        private final DoubleSupplier armSetpointSupplier;

        private double getDegrees() {
            return armSetpointSupplier.getAsDouble();
        }
    }

    @Setter
    @Getter
    private SETPOINT setpoint = SETPOINT.IDLE;

    private Rotation2d armOffset = null;

    public ArmSubsystem(ArmIO armIO) {
        this.io = armIO;

        feedback = new ProfiledPIDController(
                kP.get(),
                kI.get(),
                kD.get(),
                new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

        feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        mechanism = new Mechanism2d(1, 1);
        mechanismRoot = mechanism.getRoot("Superstructure", 0.585, 0.595);
        mechanismLigament = mechanismRoot.append(
                new MechanismLigament2d("ArmShooter", 0.5, getIdealIncrementalAngle(), 2, new Color8Bit(Color.kAqua)));
    }

    private double getIdealIncrementalAngle() {
        return armOffset == null ? 0.0 : inputs.armExternalIncrementalPosition.getDegrees() + armOffset.getDegrees();
    }

    @Override
    public void periodic() {
        // Update and process inputs
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        if (armOffset == null && inputs.armExternalAbsolutePosition.getRadians() != 0.0) {
            armOffset = inputs.armExternalAbsolutePosition.minus(inputs.armExternalIncrementalPosition);
        }

        Logger.recordOutput("Arm/Offset", armOffset.getDegrees());

        // Detect if PID gains have changed
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    feedback = new ProfiledPIDController(
                            kP.get(),
                            kI.get(),
                            kD.get(),
                            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
                    feedback.enableContinuousInput(-180, 180);
                },
                kP,
                kI,
                kD,
                maxVelocity,
                maxAcceleration);

        // Detect if feedforward gains have changed
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
                kS,
                kG,
                kV,
                kA);

        Logger.recordOutput("Arm/ExternalIncrementalAngleDegrees", inputs.armExternalIncrementalPosition.getDegrees());
        Logger.recordOutput("Arm/InternalIncrementalAngleDegrees", inputs.armInternalIncrementalPosition.getDegrees());
        Logger.recordOutput("Arm/IdealIncrementalAngleDegrees", getIdealIncrementalAngle());

        mechanismLigament.setAngle(getIdealIncrementalAngle());
        Logger.recordOutput("Arm/Mechanism2d/ArmPivot", mechanism);

        Logger.recordOutput("Arm/SetpointDegrees", setpoint.getDegrees());

        // Calculate voltage if setpoint is not idle
        if (setpoint != SETPOINT.IDLE) {
            double setpointVoltage =
                    MathUtil.clamp(feedback.calculate(getIdealIncrementalAngle(), setpoint.getDegrees()), -12, 12);
            io.setVoltage(setpointVoltage);
        } else {
            io.stopArm();
        }
    }
}
