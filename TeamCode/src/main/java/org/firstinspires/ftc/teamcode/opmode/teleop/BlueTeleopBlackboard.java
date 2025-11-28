package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;

@TeleOp(name="Blue Teleop (blackboard)", group="comp")
public class BlueTeleopBlackboard extends OpMode {
    private MainTeleop teleop;
    private Pose startPose = (Pose) blackboard.getOrDefault(RobotConstants.END_POSE_KEY, PoseConstants.BLUE_FAR_AUTO_POSE);
    @Override
    public void init() {
        teleop = new MainTeleop(startPose, Alliance.BLUE, hardwareMap, telemetry, gamepad1, false);
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
