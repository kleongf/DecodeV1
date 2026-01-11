package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
@Config
@TeleOp(name="SOTM and offset tuning teleop", group="scrim")
public class SOTMTeleop extends OpMode {
    private MainTeleop teleop;
    private Pose startPose = PoseConstants.BLUE_FAR_AUTO_POSE;
    private Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    public static double timeScaleFactor = 1;
    public static double constantTimeScaleFactor = 0.05;
    public static double offsetFactor = 0.105;
    public static double radialScaleFactor = 1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        teleop = new MainTeleop(startPose, goalPose, Alliance.BLUE, hardwareMap, telemetry, gamepad1, true);
    }

    @Override
    public void loop() {
        teleop.sotm.timeScaleFactor = timeScaleFactor;
        teleop.sotm.constantTimeFactor = constantTimeScaleFactor;
        teleop.sotm.offsetFactor = offsetFactor;
        teleop.sotm.radialVelocityScaleFactor = radialScaleFactor;
        teleop.loop();
        telemetry.update();
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
