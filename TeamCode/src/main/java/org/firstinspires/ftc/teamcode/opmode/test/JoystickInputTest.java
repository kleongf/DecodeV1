package org.firstinspires.ftc.teamcode.opmode.test;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.misc.SOTM;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;
import com.pedropathing.pathgen.Vector;
@Config
@TeleOp(name="joystick input test")
public class JoystickInputTest extends OpMode {
    @Override
    public void loop() {
        telemetry.addData("left stick x", gamepad1.left_stick_x);
        telemetry.addData("left stick y", gamepad1.left_stick_y);
        telemetry.addData("right stick x", gamepad1.right_stick_x);

        telemetry.addData("left stick x normalized", normalizeInput(gamepad1.left_stick_x));
        telemetry.addData("left stick y normalized", normalizeInput(gamepad1.left_stick_y));
        telemetry.addData("right stick x normalized", normalizeInput(gamepad1.right_stick_x));

        telemetry.update();
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    private double normalizeInput(double input) {
        return Math.signum(input) * Math.sqrt(Math.abs(input));
    }
}
