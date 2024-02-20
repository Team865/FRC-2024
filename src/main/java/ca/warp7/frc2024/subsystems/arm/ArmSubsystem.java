package ca.warp7.frc2024.subsystems.arm;

import ca.warp7.frc2024.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
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

    private final PIDController armFeedback;
    private final SimpleMotorFeedforward armFeedforward;

    private Rotation2d armSetpoint = null;

    private Mechanism2d mechanism;
    private MechanismRoot2d mechanismRoot;
    private MechanismLigament2d mechanismLigament;

    public ArmSubsystem(ArmIO armIO) {
        this.armIO = armIO;

        switch (Constants.CURRENT_MODE) {
            case REAL:
            case SIM:
                armFeedforward = new SimpleMotorFeedforward(0.1, 0.05);
                armFeedback = new PIDController(10, 0, 0);
                break;
            default:
                armFeedforward = new SimpleMotorFeedforward(0, 0);
                armFeedback = new PIDController(0, 0, 0);
                break;
        }

        mechanism = new Mechanism2d(1, 1);
        mechanismRoot = mechanism.getRoot("Superstructure", 0.585, 0.595);
        mechanismLigament =
                mechanismRoot.append(new MechanismLigament2d("ArmShooter", 0.5, 200, 2, new Color8Bit(Color.kAqua)));
    }

    public Rotation2d getAngle() {
        return armInputs.armPosition;
    }

    public void setSetpoint(Rotation2d setpoint) {
        armSetpoint = setpoint;
    }

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);

        mechanismLigament.setAngle(getAngle());

        Logger.processInputs("Arm", armInputs);
        Logger.recordOutput("Arm/Angle Deg", getAngle().getDegrees());
        Logger.recordOutput("Mechanism2d/ArmShooter", mechanism);

        if (armSetpoint != null) {
            Logger.recordOutput("Arm/Setpoint Deg", armSetpoint.getDegrees());

            double armVoltage = armFeedback.calculate(getAngle().getRadians(), armSetpoint.getRadians());
            armIO.setArmVoltage(armVoltage);

            Logger.recordOutput("Arm/Voltage", armVoltage);
        }
    }
}
