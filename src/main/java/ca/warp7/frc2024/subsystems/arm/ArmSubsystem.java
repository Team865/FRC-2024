package ca.warp7.frc2024.subsystems.arm;

import static ca.warp7.frc2024.subsystems.arm.ArmConstants.*;

import ca.warp7.frc2024.subsystems.arm.ArmConstants.Goal;
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
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
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

    /* Interpolation */
    private PolynomialCurveFitter curveFitter = PolynomialCurveFitter.create(3);
    private PolynomialFunction polynomialFunction;
    protected double distance = 0;

    /* Setpoints */
    protected Goal currentGoal = Goal.IDLE;
    private double goalDegrees;

    private Rotation2d armOffset = null;

    protected boolean lockout = false;

    public ArmSubsystem(ArmIO armIO) {
        this.io = armIO;

        double[] coefficients = curveFitter.fit(ArmConstants.POINTS);

        polynomialFunction = new PolynomialFunction(coefficients);
        Logger.recordOutput("Arm/RegressionCoefficients", coefficients);

        feedback = new ProfiledPIDController(
                kP.get(),
                kI.get(),
                kD.get(),
                new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
        feedback.setTolerance(2.5);

        feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        mechanism = new Mechanism2d(1, 1);
        mechanismRoot = mechanism.getRoot("Superstructure", 0.585, 0.595);
        mechanismLigament = mechanismRoot.append(new MechanismLigament2d(
                "ArmShooter", 0.5, getIdealIncrementalAngle().getDegrees(), 2, new Color8Bit(Color.kAqua)));

        // Arm starts stowed
        currentGoal = Goal.HANDOFF_INTAKE;
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

        // Calculate offset from absolute encoder
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
        Logger.recordOutput(
                "Arm/IdealIncrementalAngleDegrees", getIdealIncrementalAngle().getDegrees());
        Logger.recordOutput("Arm/OffsetDegrees", armOffset.getDegrees());

        // Log mechanism2d
        mechanismLigament.setAngle(getIdealIncrementalAngle().getDegrees());
        Logger.recordOutput("Arm/Mechanism2d/ArmPivot", mechanism);

        // Calculate and set voltage if goal is not idle
        // If lockout is engaged, the goal is set to the stow position'

        if (lockout || currentGoal == Goal.IDLE) {
            goalDegrees = Goal.HANDOFF_INTAKE.getDegrees();
        } else if (currentGoal == Goal.INTERPOLATION) {
            goalDegrees = polynomialFunction.value(distance);
        } else {
            goalDegrees = currentGoal.getDegrees();
        }

        goalDegrees = MathUtil.clamp(goalDegrees, 0, 81);
        Logger.recordOutput("Arm/GoalDegrees", goalDegrees);
        double goalVoltage = feedback.calculate(getIdealIncrementalAngle().getDegrees(), goalDegrees);
        io.setVoltage(goalVoltage);
    }

    protected boolean atGoal(Goal goal) {
        return this.currentGoal == goal && feedback.atGoal();
    }
}
