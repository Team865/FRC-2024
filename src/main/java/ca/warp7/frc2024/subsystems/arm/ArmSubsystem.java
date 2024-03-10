package ca.warp7.frc2024.subsystems.arm;

import static ca.warp7.frc2024.Constants.ARM.*;

import ca.warp7.frc2024.util.LoggedTunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

// TODO: Use absolute encoder if greater than x degrees

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
    private static final LoggedTunableNumber maxVelocity =
            new LoggedTunableNumber("Arm/Constraint/MaxVelocity", MAX_VELOCITY_DEG);
    private static final LoggedTunableNumber maxAcceleration =
            new LoggedTunableNumber("Arm/Constraint/MaxAcceleration", MAX_ACCELERATION_DEG);

    /* Setpoints */
    @RequiredArgsConstructor
    public enum Setpoint {
        HANDOFF_INTAKE(new LoggedTunableNumber("Arm/Setpoint/HandoffIntakeDegrees", 0)),
        STATION_INTAKE(new LoggedTunableNumber("Arm/Setpoint/StationIntakeDegrees", 0)),
        AMP(new LoggedTunableNumber("Arm/Setpoint/AmpDegrees", 67)),
        TRAP(new LoggedTunableNumber("Arm/Setpoint/TrapDegrees", 3)),
        PODIUM(new LoggedTunableNumber("Arm/Setpoint/PodiumDegrees", 65)),
        SUBWOOFER(new LoggedTunableNumber("Arm/Setpoint/SubwooferDegrees", 50)),
        BLOCKER(new LoggedTunableNumber("Arm/Setpoint/BlockerDegrees", 80)),
        IDLE(() -> 0);

        private final DoubleSupplier armSetpointSupplier;

        private double getDegrees() {
            return armSetpointSupplier.getAsDouble();
        }

        private double getRadians() {
            return Units.degreesToRadians(getDegrees());
        }
    }

    private Setpoint setpoint = Setpoint.IDLE;

    private Rotation2d armOffset = null;

    public ArmSubsystem(ArmIO armIO) {
        this.io = armIO;

        feedback = new ProfiledPIDController(
                kP.get(),
                kI.get(),
                kD.get(),
                new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
        feedback.setTolerance(1.5);

        feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        mechanism = new Mechanism2d(1, 1);
        mechanismRoot = mechanism.getRoot("Superstructure", 0.585, 0.595);
        mechanismLigament = mechanismRoot.append(new MechanismLigament2d(
                "ArmShooter", 0.5, getIdealIncrementalAngle().getDegrees(), 2, new Color8Bit(Color.kAqua)));

        setpoint = Setpoint.HANDOFF_INTAKE;
    }

    private Rotation2d getIdealIncrementalAngle() {
        return armOffset == null ? new Rotation2d() : inputs.armExternalIncrementalPosition;
        // Don't use absolute encoder bc it changes every time we start up
        // Rotation2d.fromRadians(inputs.armExternalIncrementalPosition.getRadians()+ armOffset.getRadians());
        // When adding Rotation2d objects the values wrap at -180, +180;
        // No clue how to fix, converting to degrees is a workaround
    }

    @Override
    public void periodic() {
        // Update and process inputs
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        if (armOffset == null) {
            armOffset = inputs.armExternalAbsolutePosition.minus(inputs.armExternalIncrementalPosition);
            feedback.reset(getIdealIncrementalAngle().getDegrees());
        }

        // Detect if PID gains have changed
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    feedback.setP(kP.get());
                    feedback.setI(kI.get());
                    feedback.setD(kD.get());
                    feedback.setConstraints(new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
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

        // Log angles in degrees
        Logger.recordOutput("Arm/InternalIncrementalAngleDegrees", inputs.armInternalIncrementalPosition.getDegrees());
        Logger.recordOutput("Arm/ExternalIncrementalAngleDegrees", inputs.armExternalIncrementalPosition.getDegrees());
        Logger.recordOutput("Arm/ExternalAbsoluteEncoderAngleDegrees", inputs.armExternalAbsolutePosition.getDegrees());
        Logger.recordOutput(
                "Arm/IdealIncrementalAngleDegrees", getIdealIncrementalAngle().getDegrees());
        Logger.recordOutput("Arm/OffsetDegrees", armOffset.getDegrees());
        Logger.recordOutput("Arm/SetpointDegrees", setpoint.getDegrees());

        // Log mechanism2d
        mechanismLigament.setAngle(getIdealIncrementalAngle().getDegrees());
        Logger.recordOutput("Arm/Mechanism2d/ArmPivot", mechanism);

        // Calculate and  voltage if setpoint is not idle
        if (setpoint != Setpoint.IDLE) {
            double setpointVoltage =
                    feedback.calculate(getIdealIncrementalAngle().getDegrees(), setpoint.getDegrees());

            Logger.recordOutput("Arm/FeedbackVoltage", setpointVoltage);
            Logger.recordOutput("Arm/SetpointDifference", mechanism);
            io.setVoltage(setpointVoltage);
        }
    }

    private boolean atSetpoint(Setpoint setpoint) {
        return this.setpoint == setpoint && feedback.atGoal() ? true : false;
    }

    public Trigger atSetpointTrigger(Setpoint setpoint) {
        return new Trigger(() -> atSetpoint(setpoint)).debounce(0.1);
    }

    public Command setSetpointCommand(Setpoint setpoint) {
        return this.runOnce(() -> this.setpoint = setpoint);
    }
}
