package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightLocalizer {
    private final Limelight3A limelight;
    private final double errorThreshold = 8;

    public LimelightLocalizer(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();
    }
    private double metersToInches(double meters) {
        return meters * 39.3701;
    }

    private Pose toPinpointPose(Pose3D llpose, Pose originalPose) {
        double x = 72 - (metersToInches(llpose.getPosition().x));
        double y = 72 - (metersToInches(llpose.getPosition().y));
        return new Pose(x, y, originalPose.getHeading());
    }

    public Pose update(Pose pinpointPose) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();
            Pose convertedBotPose = toPinpointPose(botPose, pinpointPose);
            // checking if they are similar
            if (Math.hypot(convertedBotPose.getX()-pinpointPose.getX(), convertedBotPose.getY()- pinpointPose.getY()) < errorThreshold) {
                return convertedBotPose;
            }
        }
        return pinpointPose;
    }

    public void start() {
        limelight.start();
    }
}