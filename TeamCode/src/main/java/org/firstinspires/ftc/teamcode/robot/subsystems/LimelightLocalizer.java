package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightLocalizer extends Subsystem {
    private final Limelight3A limelight;
    private int id = 21;
    private double heading = Math.toRadians(0);
    private Pose currentPose = new Pose(0, 0, Math.toRadians(0));

    public LimelightLocalizer(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // apriltag vision pipeline is pipeline 4
        limelight.pipelineSwitch(4);
        limelight.start();
    }

    @Override
    public void update() {
        limelight.updateRobotOrientation(heading);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                currentPose = new Pose(x, y, heading);
            }
        }
    }

    @Override
    public void start() {
        limelight.start();
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }
}