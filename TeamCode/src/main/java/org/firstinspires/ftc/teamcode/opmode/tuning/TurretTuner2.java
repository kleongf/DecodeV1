package org.firstinspires.ftc.teamcode.opmode.tuning;

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
@TeleOp(name="new turret tuning")
public class TurretTuner2 extends OpMode {
    private Turret turret;
    public static double kP = 0.004;
    public static double kD = 0.00000;
    public static double kS = 0;
    public static double target = 0;
    @Override
    public void loop() {
        turret.setPDCoefficients(kP, kD);
        turret.setKs(kS);
        turret.setTarget(Math.toRadians(target));
        turret.setFeedforward(0);

        turret.update();
        telemetry.addData("current pos", turret.turretMotor.getCurrentPosition());
        telemetry.addData("target pos", target);
        telemetry.update();

    }

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        turret.resetEncoder();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        turret.start();
    }
}
