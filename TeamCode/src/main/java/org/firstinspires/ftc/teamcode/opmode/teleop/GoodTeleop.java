package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.opmode.autonomous.BlueAutoCloseV3.END_POSE_KEY;
import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.robots.TeleopRobotV1;
import org.firstinspires.ftc.teamcode.robot.subsystems.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.misc.ClosestPoint;
import org.firstinspires.ftc.teamcode.util.misc.SOTM2;
import java.util.HashMap;
import java.util.Objects;

@TeleOp(name="first good teleop: blue")
public class GoodTeleop extends OpMode {
    private Timer localizationTimer;
    private LimelightLocalizer limelightLocalizer;
    private ClosestPoint closestPoint;
    private int state = 0;
    private boolean isAutoDriving = false;
    private Drivetrain drivetrain;
    private double longitudinalSpeed = 0.5, lateralSpeed = 0.5, rotationSpeed = 0.3;
    private TeleopRobotV1 robot;
    // TODO: try blackboard AHHHH
    private final Pose startPose = (Pose) blackboard.get(END_POSE_KEY) == null ? new Pose(54, 6, Math.toRadians(180)) : (Pose) blackboard.get(END_POSE_KEY);
    // private final Pose startPose = new Pose(54, 6, Math.toRadians(180));
    private final Pose goalPose = new Pose(0, 144, Math.toRadians(45));
    private final Pose shootPoseClose = new Pose(60, 84, Math.toRadians(180));
    private final Pose shootPoseFar = new Pose(56, 8, Math.toRadians(180));
    private SmartGamepad gp1;
    private SmartGamepad gp2;
    private SOTM2 sotm2;
    private HashMap<Integer, StateMachine> stateMap;

    @Override
    public void init() {
        // TODO: teleoprobot no reset encoder for turret
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
        localizationTimer = new Timer();
        limelightLocalizer = new LimelightLocalizer(hardwareMap);
        limelightLocalizer.start();

        closestPoint = new ClosestPoint();
    }
    @Override
    public void loop() {
        // we are moving slowly AND its been over 10 seconds
        if (localizationTimer.getElapsedTimeSeconds() > 10 && drivetrain.follower.getVelocity().getMagnitude() < 10) {
            drivetrain.follower.setCurrentPoseWithOffset(limelightLocalizer.update(drivetrain.follower.getPose()));
            localizationTimer.resetTimer();
        }

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
        // TODO: add another one to drive to latch thingy
        if (gp1.bPressed()) {
            PathChain driveToClosestPoint = drivetrain.follower.pathBuilder()
                    .addPath(
                            new Path(
                                    new BezierLine(
                                            new Point(drivetrain.follower.getPose()),
                                            new Point(closestPoint.closestPose(drivetrain.follower.getPose()))
                                    )
                            )
                    )
                    .setConstantHeadingInterpolation(drivetrain.follower.getPose().getHeading())
                    .build();
            isAutoDriving = true;
            drivetrain.follower.breakFollowing();
            drivetrain.follower.followPath(driveToClosestPoint, true);
        }

        // safety for autodrive
        if (gamepad1.leftStickButtonWasPressed() || gamepad1.rightStickButtonWasPressed()) {
            isAutoDriving = false;
            drivetrain.follower.breakFollowing();
        }

        // relocalization
        if (gp1.dpadUpPressed()) {
            drivetrain.follower.setCurrentPoseWithOffset(new Pose(138, 6, Math.toRadians(90)));
        }

        double[] values = sotm2.calculateAzimuthThetaVelocity(drivetrain.follower.getPose(), drivetrain.follower.getVelocity());

        robot.turret.setTarget(values[0]);
        robot.shooter.setShooterPitch(values[1]);
        robot.shooter.setTargetVelocity(values[2]);

        telemetry.addData("pitch", values[1]);
        telemetry.addData("velocity", values[2]);
        telemetry.addData("current velocity", robot.shooter.getCurrentVelocity());

        if (isAutoDriving) {
            // TODO: add button to automatically break following
            if (!drivetrain.follower.isBusy()) {
                isAutoDriving = false;
                drivetrain.follower.breakFollowing();
            }

        } else {
            drivetrain.setHeadingLockFieldCentricMovementVectors(-gp1.getLeftStickY()*longitudinalSpeed,
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

    @Override
    public void stop() {
        Object endPose = blackboard.getOrDefault(END_POSE_KEY, drivetrain.follower.getPose());
        blackboard.put(END_POSE_KEY, endPose);
    }
}
