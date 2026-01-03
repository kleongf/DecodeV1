package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.teleop.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.ArtifactVision;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.misc.SOTM;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;
import com.pedropathing.pathgen.Vector;
@Config
@TeleOp(name="vision test subsystem")
public class VisionTestSubsystem extends OpMode {
    private ArtifactVision vision;
    @Override
    public void loop() {
        vision.update();

        telemetry.addData("best x", vision.getLargestClusterX());
        telemetry.addData("colorlocatornull", vision.colorLocatorNull);
        telemetry.addData("has an area", vision.hasMaxArea);
        telemetry.update();
    }

    @Override
    public void init() {
        vision = new ArtifactVision(hardwareMap, Alliance.BLUE);
    }

    @Override
    public void start() {
        vision.start();
    }
}
