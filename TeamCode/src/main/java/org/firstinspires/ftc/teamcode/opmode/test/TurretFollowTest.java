package org.firstinspires.ftc.teamcode.opmode.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.misc.SOTM2;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

@TeleOp(name="turret follow test")
public class TurretFollowTest extends OpMode {
    private Turret turret;
    private Follower follower;
    private SOTM2 sotm2;
    private final Pose startPose = new Pose(54, 8, Math.toRadians(90));
    // i don't think angle matters here in the sotm calculation
    private final Pose goalPose = new Pose(12, 132, Math.toRadians(45));
    @Override
    public void loop() {
        double[] target = sotm2.calculateAzimuthThetaVelocity(follower.getPose(), follower.getVelocity());
        // set azimuth
        turret.setTarget(target[0]);
        follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        follower.update();
    }

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        turret = new Turret(hardwareMap);
        sotm2 = new SOTM2(goalPose);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }
}
