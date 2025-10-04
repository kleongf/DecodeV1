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
import org.firstinspires.ftc.teamcode.robot.subsystems.BetterIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.BetterShooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.misc.SOTM2;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;
import com.pedropathing.pathgen.Vector;
@Config
@TeleOp(name="turret follow test + shooter tuner")
public class TurretFollowTest extends OpMode {
    private Turret turret;
    public static double shooterSpeed;
    public static double shooterPitch;
    private Follower follower;
    private BetterIntake intake;
    private SOTM2 sotm2;
    private BetterShooter shooter;
    private final Pose startPose = new Pose(54, 6, Math.toRadians(180));
    // i don't think angle matters here in the sotm calculation
    private final Pose goalPose = new Pose(9, 132, Math.toRadians(45));
    @Override
    public void loop() {

        shooter.setTargetVelocity(shooterSpeed);
        // TODO: fix degree/radian disputes across shooter opmodes
        shooter.setShooterPitch(Math.toRadians(shooterPitch));
        double[] target = sotm2.calculateAzimuthThetaVelocity(follower.getPose(), new Vector());
        // set azimuth
        turret.setTarget(target[0]);
        System.out.println(target[0]);

        double dx = goalPose.getX()-follower.getPose().getX();
        double dy = goalPose.getY()-follower.getPose().getY();

        double dist = Math.hypot(dx, dy);

        // follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        follower.update();
        turret.update();
        shooter.update();
        intake.update();
        telemetry.addData("distance", dist);
        telemetry.addData("shooter speed", shooter.getCurrentVelocity());
        telemetry.update();

    }

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        turret = new Turret(hardwareMap);
        shooter = new BetterShooter(hardwareMap);
        shooter.setShooterOn(true);
        intake = new BetterIntake(hardwareMap);
        intake.setIntakeOn(true);
        intake.intakeDown();
        sotm2 = new SOTM2(goalPose);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        //follower.startTeleopDrive();
    }
}
