package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;

@TeleOp(name="Red Teleop (no blackboard)", group="comp")
public class RedTeleop extends OpMode {
    private MainTeleop teleop;
    private Pose startPose = PoseConstants.RED_FAR_AUTO_POSE;
    private Pose goalPose = PoseConstants.RED_GOAL_POSE;
    @Override
    public void init() {
        teleop = new MainTeleop(startPose, goalPose, Alliance.RED, hardwareMap, telemetry, gamepad1, false);
    }

    @Override
    public void loop() {
        teleop.loop();
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void stop() {
        teleop.stop();
    }
}
