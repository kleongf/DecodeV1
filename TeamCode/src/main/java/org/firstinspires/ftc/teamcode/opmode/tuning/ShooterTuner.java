package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.BetterShooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.misc.SOTM2;

@Config
@TeleOp(name="Shooter Tuner")
public class ShooterTuner extends OpMode {
    private Follower follower;
    private BetterShooter shooter;
    private Turret turret;
    public static double shooterAngleDegrees = 30;
    public static double shooterSpeedTicks = 0;
    private Pose startPose = new Pose(56, 6, Math.toRadians(180));
    private Pose goalPose = new Pose(12, 132, Math.toRadians(135));
    private SOTM2 sotm2;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        shooter = new BetterShooter(hardwareMap);
        turret = new Turret(hardwareMap);
        sotm2 = new SOTM2(goalPose);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void loop() {
        shooter.setTargetVelocity(shooterSpeedTicks);
        // TODO: fix degree/radian disputes across shooter opmodes
        shooter.setShooterPitch(Math.toRadians(shooterAngleDegrees));
        // uh no this is bad
        // if i want shooter to stay focused on target, we should set its heading to goal-current (right hand pos)
        double dx = goalPose.getX()-follower.getPose().getX();
        double dy = goalPose.getY()-follower.getPose().getY();

        double[] tgt = sotm2.calculateAzimuthThetaVelocity(follower.getPose(), new Vector());
        turret.setTarget(tgt[0]);

        double distanceToTarget = Math.hypot(dx, dy);

        shooter.update();
        turret.update();

        telemetry.addData("current velocity", shooter.getCurrentVelocity());
        telemetry.addData("distance to target", distanceToTarget);
        telemetry.update();

    }
}
