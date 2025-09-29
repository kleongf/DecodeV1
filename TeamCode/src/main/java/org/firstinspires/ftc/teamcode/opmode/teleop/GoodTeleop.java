package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.robots.TeleopRobot;
import org.firstinspires.ftc.teamcode.robot.robots.TeleopRobotV1;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.misc.SOTM2;
import org.firstinspires.ftc.teamcode.util.misc.Target;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.HashMap;
import java.util.Objects;

@TeleOp(name="first good teleop: blue")
public class GoodTeleop extends OpMode {
    private int state = 1;
    private boolean isAutoDriving = false;
    private Drivetrain drivetrain;
    private double longitudinalSpeed = 0.5, lateralSpeed = 0.5, rotationSpeed = 0.5;
    private TeleopRobotV1 robot;
    private final Pose startPose = new Pose(56, 6, Math.toRadians(180));
    private final Pose goalPose = new Pose(9, 132, Math.toRadians(45));
    private final Pose shootPoseClose = new Pose(60, 84, Math.toRadians(180));
    private final Pose shootPoseFar = new Pose(56, 8, Math.toRadians(180));
    private SmartGamepad gp1;
    private SmartGamepad gp2;
    private SOTM2 sotm2;
    private HashMap<Integer, StateMachine> stateMap;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setStartingPose(startPose);

        robot = new TeleopRobotV1(hardwareMap);

        gp1 = new SmartGamepad(gamepad1);
        gp2 = new SmartGamepad(gamepad2);

        sotm2 = new SOTM2(goalPose);

        stateMap = new HashMap<>();

        stateMap.put(0, robot.prepareIntake);
        stateMap.put(1, robot.prepareShooting);
        stateMap.put(2, robot.startShooting);
    }
    @Override
    public void loop() {
        gp1.update();
        gp2.update();

        if (gp1.rightBumperPressed()) {
            state++;
            Objects.requireNonNull(stateMap.get(Math.floorMod(state, 3))).start();
        }

        // very cool back button should work
        if (gp1.leftBumperPressed()) {
            state--;
            Objects.requireNonNull(stateMap.get(Math.floorMod(state, 3))).start();
        }

        if (gp1.xPressed()) {
            PathChain driveClose = drivetrain.follower.pathBuilder()
                    .addPath(
                            new Path(
                                    new BezierLine(
                                            new Point(drivetrain.follower.getPose()),
                                            new Point(shootPoseClose)
                                    )
                            )
                    )
                    .setLinearHeadingInterpolation(drivetrain.follower.getPose().getHeading(), shootPoseClose.getHeading())
                    .build();
            isAutoDriving = true;
            drivetrain.follower.breakFollowing();
            drivetrain.follower.followPath(driveClose, true);
        }

        if (gp1.yPressed()) {
            PathChain driveFar = drivetrain.follower.pathBuilder()
                    .addPath(
                            new Path(
                                    new BezierLine(
                                            new Point(drivetrain.follower.getPose()),
                                            new Point(shootPoseFar)
                                    )
                            )
                    )
                    .setLinearHeadingInterpolation(drivetrain.follower.getPose().getHeading(), shootPoseFar.getHeading())
                    .build();
            isAutoDriving = true;
            drivetrain.follower.breakFollowing();
            drivetrain.follower.followPath(driveFar, true);
        }

        double[] values = sotm2.calculateAzimuthThetaVelocity(drivetrain.follower.getPose(), drivetrain.follower.getVelocity());

        robot.turret.setTarget(values[0]);
        robot.shooter.setShooterPitch(values[1]);
        robot.shooter.setTargetVelocity(values[2]);

        telemetry.addData("pitch", values[1]);
        telemetry.addData("velocity", values[2]);
        telemetry.addData("current velocity", robot.shooter.getCurrentVelocity());

        if (isAutoDriving) {
            if (!drivetrain.follower.isBusy()) {
                isAutoDriving = false;
                drivetrain.follower.breakFollowing();
            }
        } else {
            drivetrain.setMovementVectors(gp1.getLeftStickY()*longitudinalSpeed,
                    gp1.getLeftStickX()*lateralSpeed,
                    gp1.getRightStickX()*rotationSpeed);
        }

        drivetrain.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        // TODO: In the future, initPositions() should go here so we don't move on init
        robot.initPositions();
        robot.shooter.setShooterOn(true);
        robot.start();
    }
}
