package ca.warp7.frc2024.subsystems.arm;

import ca.warp7.frc2024.Constants;
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
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    private final ProfiledPIDController armFeedback;
    private final ArmFeedforward armFeedforward;

    private Rotation2d armSetpoint = null;

    private Mechanism2d mechanism;
    private MechanismRoot2d mechanismRoot;
    private MechanismLigament2d mechanismLigament;

    public ArmSubsystem(ArmIO armIO) {
        this.armIO = armIO;

        switch (Constants.CURRENT_MODE) {
            case REAL:
                armFeedforward = new ArmFeedforward(0, 0, 1.27, 0.02);
                armFeedback = new ProfiledPIDController(.3, 0.01, 0, new TrapezoidProfile.Constraints(572, 1000));
                armFeedback.setTolerance(0.1);
                break;
            case SIM:
                armFeedforward = new ArmFeedforward(0, 0, 1.27, 0.02);
                armFeedback =
                        new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(6.2591118, 6.2591118));
                break;
            default:
                armFeedforward = new ArmFeedforward(0, 0, 0);
                armFeedback = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(10, 10));
                break;
        }

        mechanism = new Mechanism2d(1, 1);

        mechanismRoot = mechanism.getRoot("Superstructure", 0.585, 0.595);
        mechanismLigament =
                mechanismRoot.append(new MechanismLigament2d("ArmShooter", 0.5, 200, 2, new Color8Bit(Color.kAqua)));
    }

    public Rotation2d getExternalIncrementalPosition() {
        return armInputs.armExternalIncrementalPosition;
    }

    public Rotation2d getExternalAbsolutePosition() {
        return armInputs.armExternalAbsolutePosition;
    }

    public Rotation2d getInternalIncrementalPosition() {
        return armInputs.armInternalIncrementalPosition;
    }

    public void runSetpoint(Rotation2d setpoint) {
        armSetpoint = setpoint;
    }

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        Logger.processInputs("Arm", armInputs);

        Logger.recordOutput("Arm/ExternalIncrementalAngleDegs", armInputs.armExternalIncrementalPosition.getDegrees());
        Logger.recordOutput("Arm/InternalIncrementalAngleDegs", armInputs.armInternalIncrementalPosition.getDegrees());

        mechanismLigament.setAngle(getInternalIncrementalPosition());
        Logger.recordOutput("Mechanism2d/ArmShooter", mechanism);

        if (armSetpoint != null) {
            Logger.recordOutput("Arm/Setpoint Deg", armSetpoint.getDegrees());

            double armVoltage =
                    armFeedback.calculate(getExternalIncrementalPosition().getDegrees(), armSetpoint.getDegrees());
            armIO.setArmVoltage(armVoltage);

            Logger.recordOutput("Arm/Voltage", armVoltage);
        }
    }
}
