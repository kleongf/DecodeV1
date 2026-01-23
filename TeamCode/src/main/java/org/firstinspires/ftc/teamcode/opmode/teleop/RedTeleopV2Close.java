package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;

@TeleOp(name="Red Teleop V2 Close (no reset encoders)", group="!")
public class RedTeleopV2Close extends OpMode {
    private MainTeleopV2 teleop;
    private Pose startPose = (Pose) blackboard.getOrDefault(RobotConstants.END_POSE_KEY, PoseConstants.RED_END_AUTO_POSE);
    private Pose goalPose = PoseConstants.RED_GOAL_POSE;
    @Override
    public void init() {
        teleop = new MainTeleopV2(startPose, goalPose, Alliance.RED, hardwareMap, telemetry, gamepad1, false);
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
