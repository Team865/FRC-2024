package ca.warp7.frc2024.subsystems.drivetrain;

import static ca.warp7.frc2024.Constants.OI.*;
import static ca.warp7.frc2024.subsystems.drivetrain.DrivetrainConstants.*;
import static edu.wpi.first.units.Units.Volts;

import ca.warp7.frc2024.FieldConstants.PointOfInterest;
import ca.warp7.frc2024.subsystems.vision.VisionIO;
import ca.warp7.frc2024.subsystems.vision.VisionIOInputsAutoLogged;
import ca.warp7.frc2024.util.LoggedTunableNumber;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrivetrainSubsystem extends SubsystemBase {
    /* AdvantageKit */
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final VisionIO frontVisionIO;
    private final VisionIOInputsAutoLogged frontVisionInputs = new VisionIOInputsAutoLogged();

    private final VisionIO rearVisionIO;
    private final VisionIOInputsAutoLogged rearVisionInputs = new VisionIOInputsAutoLogged();

    /* Drivetrain */
    private final SwerveModule[] swerveModules = new SwerveModule[4];
    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(SWERVE_MODULE_TRANSLATIONS);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            swerveDriveKinematics,
            rawGyroRotation,
            lastModulePositions,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, 0.005),
            VecBuilder.fill(2, 2, 999999999));

    /* Controllers */
    protected final PIDController aimAtFeedback;
    protected final PIDController headingSnapFeedback;

    /* Gains */
    private final LoggedTunableNumber aimAtkP = new LoggedTunableNumber("Drivetrain/Gains/AimAt/kP", 0.15);
    private final LoggedTunableNumber aimAtkI = new LoggedTunableNumber("Drivetrain/Gains/AimAt/kI", 0);
    private final LoggedTunableNumber aimAtkD = new LoggedTunableNumber("Drivetrain/Gains/AimAt/kD", 0);

    private final LoggedTunableNumber rotateTokP = new LoggedTunableNumber("Drivetrain/Gains/HeadingSnap/kP", 0.15);
    private final LoggedTunableNumber rotateTokI = new LoggedTunableNumber("Drivetrain/Gains/HeadingSnap/kI", 0);
    private final LoggedTunableNumber rotateTokD = new LoggedTunableNumber("Drivetrain/Gains/HeadingSnap/kD", 0);

    protected final LoggedTunableNumber fudge = new LoggedTunableNumber("Drivetrain/Fudge", 0.1);

    private final LoggedTunableNumber drivekS = new LoggedTunableNumber("Drivetrain/Gains/Drive/kS", DRIVE_GAINS.kS());
    private final LoggedTunableNumber drivekV = new LoggedTunableNumber("Drivetrain/Gains/Drive/kV", DRIVE_GAINS.kV());

    private final LoggedTunableNumber drivekP = new LoggedTunableNumber("Drivetrain/Gains/Drive/kP", DRIVE_GAINS.kP());
    private final LoggedTunableNumber drivekI = new LoggedTunableNumber("Drivetrain/Gains/Drive/kI", DRIVE_GAINS.kI());
    private final LoggedTunableNumber drivekD = new LoggedTunableNumber("Drivetrain/Gains/Drive/kD", DRIVE_GAINS.kD());

    private final LoggedTunableNumber steerkP = new LoggedTunableNumber("Drivetrain/Gains/Steer/kP", STEER_GAINS.kP());
    private final LoggedTunableNumber steerkI = new LoggedTunableNumber("Drivetrain/Gains/Steer/kI", STEER_GAINS.kI());
    private final LoggedTunableNumber steerkD = new LoggedTunableNumber("Drivetrain/Gains/Steer/kD", STEER_GAINS.kD());

    protected final SysIdRoutine sysId;

    protected PointOfInterest pointAt = PointOfInterest.NONE;
    protected boolean reversePointing = true;

    protected HeadingSnapPoint headingSnapPoint = HeadingSnapPoint.NONE;

    protected double speedMultiplier = 1;

    public SwerveDrivetrainSubsystem(
            GyroIO gyroIO,
            VisionIO frontVisionIO,
            VisionIO rearVisionIO,
            SwerveModuleIO frontRightSwerveModuleIO,
            SwerveModuleIO frontLeftSwerveModuleIO,
            SwerveModuleIO backLeftSwerveModuleIO,
            SwerveModuleIO backRightSwerveModuleIO) {
        this.gyroIO = gyroIO;
        this.frontVisionIO = frontVisionIO;
        this.rearVisionIO = rearVisionIO;

        swerveModules[0] = new SwerveModule(frontRightSwerveModuleIO, "FrontRight");
        swerveModules[1] = new SwerveModule(frontLeftSwerveModuleIO, "FrontLeft");
        swerveModules[2] = new SwerveModule(backLeftSwerveModuleIO, "BackLeft");
        swerveModules[3] = new SwerveModule(backRightSwerveModuleIO, "BackRight");

        headingSnapFeedback = new PIDController(rotateTokP.get(), rotateTokI.get(), rotateTokD.get());
        headingSnapFeedback.enableContinuousInput(-180, 180);

        // Create and configure feedback controller
        aimAtFeedback = new PIDController(aimAtkP.get(), aimAtkI.get(), aimAtkD.get());
        aimAtFeedback.enableContinuousInput(-180, 180);

        // Reset robot pose
        setPose(new Pose2d(0, 0, new Rotation2d()));

        // Set initial gains
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDriveFeedforwardGains(drivekS.get(), drivekV.get());
            swerveModules[i].setDriveFeedbackGains(drivekP.get(), drivekI.get(), drivekD.get());
            swerveModules[i].setSteerFeedbackGains(steerkP.get(), steerkI.get(), steerkD.get());
        }

        // Configure PathPlanner
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                () -> swerveDriveKinematics.toChassisSpeeds(getModuleStates()),
                this::setTargetChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(2.5),
                        new PIDConstants(2.5),
                        MAX_LINEAR_SPEED,
                        DRIVE_BASE_RADIUS,
                        new ReplanningConfig(true, true, 1, 0.05)),
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red,
                this);

        // Log PathPlanner
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Drivetrain/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Drivetrain/Odometry/TrajectoryTarget", targetPose);
        });

        // Configure SysID Routine
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Drivetrain/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            for (int i = 0; i < 4; i++) {
                                swerveModules[i].runCharacterization(voltage.in(Volts));
                            }
                        },
                        null,
                        this));
    }

    @Override
    public void periodic() {
        // Update and process inputs
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drivetrain/Gyro", gyroInputs);

        frontVisionIO.updateInputs(frontVisionInputs);
        Logger.processInputs("Drivetrain/Vision/Front", frontVisionInputs);

        rearVisionIO.updateInputs(rearVisionInputs);
        Logger.processInputs("Drivetrain/Vision/Rear", rearVisionInputs);

        // Run swerve module periodic routines
        for (var module : swerveModules) {
            module.periodic();
        }

        // Calculate last module position
        SwerveModulePosition[] modulePositions = getModulePositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            moduleDeltas[i] = new SwerveModulePosition(
                    modulePositions[i].distanceMeters - lastModulePositions[i].distanceMeters,
                    modulePositions[i].angle);
            lastModulePositions[i] = modulePositions[i];
        }

        // Use real inputs if gyro exists, otherwise generate from kinematics
        if (gyroInputs.gyroConnected) {
            rawGyroRotation = gyroInputs.gyroYaw;
        } else {
            Twist2d twist = swerveDriveKinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // Update pose estimator using odometry
        poseEstimator.update(rawGyroRotation, modulePositions);

        // if (rearVisionInputs.tagCount >= 2 && rearVisionInputs.avgTagDist <= 4.5) {
        //     poseEstimator.addVisionMeasurement(
        //             rearVisionInputs.blueOriginRobotPose, rearVisionInputs.timestamp, VecBuilder.fill(1, 1,
        // 999999999));
        // }

        updatePoseEstimateWithVision();

        Logger.recordOutput("Drivetrain/DistanceToSpeakerWall", getDistanceToPOI(PointOfInterest.SPEAKER_WALL));
        Logger.recordOutput("Drivetrain/DistanceToAmp", getDistanceToPOI(PointOfInterest.AMP));

        // Update if PID gains have changed
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    aimAtFeedback.setP(aimAtkP.get());
                    aimAtFeedback.setI(aimAtkI.get());
                    aimAtFeedback.setD(aimAtkD.get());
                },
                aimAtkP,
                aimAtkI,
                aimAtkD);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    for (int i = 0; i < 4; i++) {
                        swerveModules[i].setDriveFeedforwardGains(drivekS.get(), drivekV.get());
                    }
                },
                drivekS,
                drivekV);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    for (int i = 0; i < 4; i++) {
                        swerveModules[i].setDriveFeedbackGains(drivekP.get(), drivekI.get(), drivekD.get());
                    }
                },
                drivekP,
                drivekI,
                drivekD);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    for (int i = 0; i < 4; i++) {
                        swerveModules[i].setSteerFeedbackGains(steerkP.get(), steerkI.get(), steerkD.get());
                    }
                },
                steerkP,
                steerkI,
                steerkD);
    }

    public void updatePoseEstimateWithVision() {
        if (rearVisionInputs.tagCount >= 1) {
            double xyStds;
            double rotStds = 999999999;

            if (rearVisionInputs.tagCount >= 2 && rearVisionInputs.avgTagDist <= 3.65) {
                xyStds = 0.5;
            } else if (rearVisionInputs.avgTagDist < 2) {
                xyStds = 1.5;
            } else {
                return;
            }

            poseEstimator.addVisionMeasurement(
                    rearVisionInputs.blueOriginRobotPose,
                    rearVisionInputs.timestamp,
                    VecBuilder.fill(xyStds, xyStds, rotStds));
        }
    }
    /**
     * Drive at desired velocities
     *
     * @param speeds Velocities in ChassisSpeeds object
     */
    public void setTargetChassisSpeeds(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_LINEAR_SPEED);

        // Set individual modules target state
        SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            optimizedStates[i] = swerveModules[i].setTargetState(swerveModuleStates[i]);
        }

        // Log states
        Logger.recordOutput("Drivetrain/Swerve/TargetStates", swerveModuleStates);
        Logger.recordOutput("Drivetrain/Swerve/TargetStatesOptimized", optimizedStates);
    }

    /**
     * @return Current pose from pose estimator
     */
    @AutoLogOutput(key = "Drivetrain/Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Drivetrain/DistanceToPointOfInterest")
    public double getDistanceToPOI(PointOfInterest POI) {
        return getPose().getTranslation().getDistance(POI.getAllianceTranslation());
    }

    /**
     * @return Current rotation from pose estimator
     */
    public Rotation2d getRobotRotation() {
        return getPose().getRotation();
    }

    /**
     * Resets pose estimator to provided pose
     * @param pose
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * @return An array of module positions
     */
    @AutoLogOutput(key = "Drivetrain/Swerve/Positions")
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = swerveModules[i].getPosition();
        }
        return positions;
    }

    /**
     * @return An array of module states
     */
    @AutoLogOutput(key = "Drivetrain/Swerve/States")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    /**
     * Stop the drivetrain by setting empty chassis speed
     */
    protected void stop() {
        setTargetChassisSpeeds(new ChassisSpeeds());
    }

    /**
     * Puts swerve wheels in an x pattern to prevent movement
     */
    protected void stopWithX() {
        swerveModules[0].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        swerveModules[1].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
        swerveModules[2].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        swerveModules[3].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
    }

    /**
     * Zeroes encoder on a hardware level
     */
    protected void zeroGyro() {
        gyroIO.zeroYaw();
    }
}
