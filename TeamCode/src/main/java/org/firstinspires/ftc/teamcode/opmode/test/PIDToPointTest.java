package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.purepursuit.Pose2D;
import org.firstinspires.ftc.teamcode.util.purepursuit.PurePursuit;

// idea behind this code: if we can tune our pid to hold a point and go back to it we are chilling
@Config
@TeleOp
public class PIDToPointTest extends OpMode {
    private PurePursuit follower;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose2D poseToHold = new Pose2D(0, 0, Math.toRadians(0));
    public static double pDrive = 0;
    public static double dDrive = 0;
    public static double pTranslational = 0;
    public static double dTranslational = 0;
    public static double pHeading = 0;
    public static double dHeading = 0;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new PurePursuit(hardwareMap);
        follower.setStartingPose(startPose);
        follower.pidToPoint(poseToHold);
    }
    @Override
    public void loop() {
        follower.longitudinalController.setPIDF(pDrive, 0, dDrive, 0);
        follower.lateralController.setPIDF(pTranslational, 0, dTranslational, 0);
        follower.headingController.setPIDF(pHeading, 0, dHeading, 0);
        follower.update();
        telemetry.update();
    }
}
