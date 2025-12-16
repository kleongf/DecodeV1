package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
@Config
@TeleOp(name="SOTM tuning teleop", group="scrim")
public class SOTMTeleop extends OpMode {
    private MainTeleop teleop;
    private Pose startPose = PoseConstants.BLUE_FAR_AUTO_POSE;
    private Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    public static double timeScaleFactor = 1;
    @Override
    public void init() {
        teleop = new MainTeleop(startPose, goalPose, Alliance.BLUE, hardwareMap, telemetry, gamepad1, true);
    }

    @Override
    public void loop() {
        teleop.sotm.setTimeScaleFactor(timeScaleFactor);
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
