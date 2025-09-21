package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.robots.TeleopRobot;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.misc.Target;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
@TeleOp(name="first teleop BAD")
public class FirstTeleop extends OpMode {
    private VoltageCompFollower follower;
    private boolean isAutoDriving = false;
    private double longitudinalSpeed = 1.0, lateralSpeed = 1.0, rotationSpeed = 1.0;
    private TeleopRobot robot;
    private final Pose startPose = new Pose(32, 7, Math.toRadians(90));
    private SmartGamepad gp1;
    private SmartGamepad gp2;
    private Target region1 = new Target(54, 96, Math.toRadians(135), 300, 45);
    private Target region2 = new Target(34, 116, Math.toRadians(135), 150, 65);
    private Target region3 = new Target(74, 76, Math.toRadians(135), 400, 40);
    // not using 4 for now
    private Target region4 = new Target(58, 120, Math.toRadians(155), 300, 45);


    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        robot = new TeleopRobot(hardwareMap);

        gp1 = new SmartGamepad(gamepad1);
        gp2 = new SmartGamepad(gamepad2);

        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.initPositions();
    }
    @Override
    public void loop() {
        if (isAutoDriving && !follower.isBusy()) {
            isAutoDriving = false;
            follower.startTeleopDrive();
        }
        // TODO: have a robot is busy thing so that commands dont collide
        gp1.update();
        gp2.update();

        if (gp1.leftBumperPressed()) {
            robot.prepareIntake.start();
        }

        if (gp1.rightBumperPressed()) {
            robot.prepareShooting.start();
        }

        if (gp1.xPressed()) {
            robot.shootGreen.start();
        }

        if (gp1.yPressed()) {
            robot.shootPurple.start();
        }

        if (gp2.dpadUpPressed()) {
            PathChain driveToR1 = follower.pathBuilder()
                    .addPath(
                            new Path(
                                    new BezierLine(
                                        new Point(follower.getPose()),
                                        new Point(region1.getxPos(), region1.getyPos(), Point.CARTESIAN)
                                    )
                            )
                    )
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), region1.getHeading())
                    .build();
            isAutoDriving = true;
            follower.breakFollowing();
            follower.followPath(driveToR1);
            robot.shooter.setShooterPitchDegrees(region1.getShooterPitch());
            robot.shooter.setTargetVelocity(region1.getShooterVelocity());
        }

        // left and right for the left and right respectively

        follower.update();
        if (!isAutoDriving) {
            follower.setTeleOpMovementVectors(
                    gp1.getLeftStickY() * longitudinalSpeed,
                    gp1.getLeftStickX() * lateralSpeed,
                    gp1.getRightStickX() * rotationSpeed
            );
        }
        robot.update();
    }

    @Override
    public void start() {
        // TODO: In the future, initPositions() should go here so we don't move on init
        follower.startTeleopDrive();
        robot.start();
    }
}
