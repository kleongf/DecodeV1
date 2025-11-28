package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

@Config
@TeleOp(name="limelight pose test")
public class LimelightPoseTest extends OpMode {
    // this assumes the limelight is like 3 inches right from center of robot, up like 9 inches, and angled at 45
    private Limelight3A limelight;
    private Follower follower;
    private Intake intake;
    private final Pose startPose = new Pose(30, 137, Math.toRadians(270));

    private double metersToInches(double meters) {
        return meters * 39.3701;
    }

    private String pedroPose(Pose3D llpose) {
        double x = 72 + (metersToInches(llpose.getPosition().y));
        double y = 72 - (metersToInches(llpose.getPosition().x));
        return "X: " + x + ", Y: " + y;
    }

    @Override
    public void loop() {
        follower.update();
        intake.update();
        LLResult result = limelight.getLatestResult();
        telemetry.addData("result is null?", result == null);
        telemetry.addData("result is valid?", result.isValid());
        if (result != null) {
            Pose3D botpose = result.getBotpose();
            System.out.println(botpose.toString());
            telemetry.addData("pose", botpose.toString());
            telemetry.addData("pedro converted pose", pedroPose(botpose));
            telemetry.addData("actual pinpoint pose", follower.getPose());
        }

        telemetry.update();
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight.start();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        intake = new Intake(hardwareMap);
        intake.state = Intake.IntakeState.INTAKE_FAST;
    }

    @Override
    public void start() {
        limelight.start();
    }
}