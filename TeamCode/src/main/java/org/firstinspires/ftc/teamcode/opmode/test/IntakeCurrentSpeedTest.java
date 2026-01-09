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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

@Config
@TeleOp(name="intake current and speed test")
public class IntakeCurrentSpeedTest extends OpMode {
    private Intake intake;

    @Override
    public void loop() {
        intake.update();
        if (gamepad1.x) {
            intake.state = Intake.IntakeState.INTAKE_FAST;
        }
        if (gamepad1.y) {
            intake.state = Intake.IntakeState.INTAKE_OFF;
        }

        telemetry.addData("Intake current", intake.intakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Intake speed", intake.intakeMotor.getVelocity());

        telemetry.update();
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(hardwareMap);
        intake.state = Intake.IntakeState.INTAKE_FAST;
    }

    @Override
    public void start() {

    }
}
