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
    private double xOffset = 0;
    private double yOffset = 0;

    public LimelightLocalizer(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();
    }
    private double metersToInches(double meters) {
        return meters * 39.3701;
    }

    private Pose toPinpointPose(Pose3D llpose, Pose pinpointPose) {
        double x = 72 + (metersToInches(llpose.getPosition().y));
        double y = 72 - (metersToInches(llpose.getPosition().x));
        return new Pose(x, y, pinpointPose.getHeading());
    }

    private double[] xy(Pose3D llpose) {
        double x = 72 + (metersToInches(llpose.getPosition().y));
        double y = 72 - (metersToInches(llpose.getPosition().x));
        return new double[] {x, y};
    }

    public Pose update(Pose pinpointPose) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();
            Pose convertedBotPose = toPinpointPose(botPose, pinpointPose);
            Pose offsetPose = new Pose(convertedBotPose.getX() + xOffset, convertedBotPose.getY() + yOffset);
            // checking if they are similar
            if (offsetPose.getX() == 72 && offsetPose.getY() == 72) {
                return pinpointPose;
            }

            if ((Math.hypot(offsetPose.getX()-pinpointPose.getX(), offsetPose.getY()-pinpointPose.getY()) < errorThreshold)) {
                return offsetPose;
            }
        }
        return pinpointPose;
    }

    public Pose overrideUpdate(Pose pinpointPose) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();
            Pose convertedBotPose = toPinpointPose(botPose, pinpointPose);
            // oh i know why relocalization was failing now. forgot to set heading lol
            Pose offsetPose = new Pose(convertedBotPose.getX() + xOffset, convertedBotPose.getY() + yOffset, Math.toRadians(botPose.getPosition().z));
            return offsetPose;
        }
        return pinpointPose;
    }

    public double[] getXY() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();
            double[] xy = xy(botPose);
            return xy;
        }
        return new double[] {0, 0};
    }

    public void setXYOffset(double xOffset, double yOffset) {
        this.xOffset = xOffset;
        this.yOffset = yOffset;
    }

    public void start() {
        limelight.start();
    }
}