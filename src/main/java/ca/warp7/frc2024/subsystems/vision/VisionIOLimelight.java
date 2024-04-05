package ca.warp7.frc2024.subsystems.vision;

import ca.warp7.frc2024.util.LimelightHelpers;
import ca.warp7.frc2024.util.LimelightHelpers.PoseEstimate;

public class VisionIOLimelight implements VisionIO {

    private String llName;

    public VisionIOLimelight(String limelightName) {
        this.llName = limelightName;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName);

        inputs.blueOriginRobotPose = poseEstimate.pose;
        inputs.timestamp = poseEstimate.timestampSeconds;
        inputs.latency = poseEstimate.latency;
        inputs.tagCount = poseEstimate.tagCount;
        inputs.tagSpan = poseEstimate.tagSpan;
        inputs.avgTagDist = poseEstimate.avgTagDist;
        inputs.avgTagArea = poseEstimate.avgTagArea;
        inputs.name = llName;
    }

    @Override
    public void setPipeline(int index) {
        LimelightHelpers.setPipelineIndex(llName, index);
    }

    @Override
    public void setStreamStandard() {
        LimelightHelpers.setStreamMode_PiPMain(llName);
    }

    @Override
    public void setStreamPiPMain() {
        LimelightHelpers.setStreamMode_PiPMain(llName);
    }

    @Override
    public void setStreamPiPSecondary() {
        LimelightHelpers.setStreamMode_PiPMain(llName);
    }

    @Override
    public void setRobotOrientation(double yaw) {
        LimelightHelpers.SetRobotOrientation(llName, yaw, 0, 0, 0, 0, 0);
    }
}
