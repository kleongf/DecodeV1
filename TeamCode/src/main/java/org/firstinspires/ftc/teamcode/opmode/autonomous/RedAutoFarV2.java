package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.SOTM2;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

@Autonomous(name="legal red auto far v5: 18, shoot close", group="comp")
public class RedAutoFarV2 extends OpMode {
    public static final String END_POSE_KEY = "END_POSE";
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM2 sotm2;
    private final Pose startPose = new Pose(90, 6, Math.toRadians(90));
    private final Pose shootPoseNear = new Pose(84, 84, Math.toRadians(0));
    private final Pose shootPoseNearAngled = new Pose(84, 84, Math.toRadians(300));

    private final Pose goalPose = new Pose(144, 144, Math.toRadians(45));

    private PathChain shootPreload, intakeFirst, shootFirst, intakeThird, shootThird, intakeSecond, openGate, shootSecond, intakeCorner1, shootCorner1, intakeCorner2, shootCorner2;
    public void buildPaths() {
        shootPreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(90, 6), new Pose(84, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();
        intakeFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 84.000), new Pose(124.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        shootFirst = follower.pathBuilder()
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(124.000, 84.000), new Pose(84.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        intakeThird = follower.pathBuilder()
                .addPath(
                        // Path 4
                        new BezierCurve(
                                new Pose(84.000, 84.000),
                                new Pose(144-51.319, 30.064),
                                new Pose(144-46.532, 30.255),
                                new Pose(131.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        shootThird = follower.pathBuilder()
                .addPath(
                        // Path 5
                        new BezierCurve(
                                new Pose(131.000, 36.000),
                                new Pose(144-62.617, 45.191),
                                new Pose(84.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        intakeSecond = follower.pathBuilder()
                .addPath(
                        // Path 6
                        new BezierCurve(
                                new Pose(84.000, 84.000),
                                new Pose(144-47.681, 55.021),
                                new Pose(144-41.170, 55.213),
                                new Pose(131.000, 58.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        openGate = follower.pathBuilder()
                .addPath(
                        // Path 7
                        new BezierCurve(
                                new Pose(131.000, 58.000),
                                new Pose(144-27.000, 55.915),
                                new Pose(110.000, 69.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .addPath(
                        new BezierCurve(
                                new Pose(110, 69),
                                new Pose(130, 69)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();
        shootSecond = follower.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(130.000, 69.000), new Pose(84.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(300))
                .build();
        intakeCorner1 = follower.pathBuilder()
                .addPath(
                        // Path 2
                        new BezierLine(new Pose(84.000, 84.000), new Pose(132, 10.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(300))
                .build();
        shootCorner1 = follower.pathBuilder()
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(132, 10.000), new Pose(84, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(300))
                .build();
        intakeCorner2 = follower.pathBuilder()
                .addPath(
                        // Path 4
                        new BezierLine(new Pose(84.000, 84.000), new Pose(132.000, 10.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(300))
                .build();
        shootCorner2 = follower.pathBuilder()
                .addPath(
                        // Path 5
                        new BezierLine(new Pose(132.000, 10.000), new Pose(84.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(300))
                .build();
//        park = follower.pathBuilder()
//                .addPath(
//                        // Path 12
//                        new BezierLine(new Pose(54, 10), new Pose(34.000, 10.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
    }

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap);
        sotm2 = new SOTM2(goalPose);
        sotm2.isBlue = false;
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPreload, true);
                            robot.prepareShooting.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeFirst, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            follower.followPath(shootFirst, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeThird, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            follower.followPath(shootThird, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeSecond, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            follower.followPath(openGate, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            double[] values = sotm2.calculateAzimuthThetaVelocity(shootPoseNearAngled, new Vector());
                            robot.turret.setTarget(values[0]);
                            robot.shooter.setShooterPitch(values[1]);
                            robot.shooter.setTargetVelocity(values[2]);
                        })
                        .maxTime(670), // when there are 9 balls a hard hit will do
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeCorner1, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            follower.followPath(shootCorner1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeCorner2, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            follower.followPath(shootCorner2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished()))
//                new State()
//                        .onEnter(() -> follower.followPath(park, true))
//                        .transition(new Transition(() -> !follower.isBusy()))
        );

        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.initPositions();
    }
    @Override
    public void loop() {
        stateMachine.update();
        follower.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        // constant values. shooter is never turned off after start.
        double[] values = sotm2.calculateAzimuthThetaVelocity(new Pose(84, 84, Math.toRadians(0)), new Vector());

        robot.turret.setTarget(values[0]);
        robot.shooter.setShooterPitch(values[1]);
        // velocity is higher than tuned because needs to hit backboard because shooter sucks
        robot.shooter.setTargetVelocity(values[2]);
        robot.shooter.setShooterOn(true);

        stateMachine.start();
        robot.start();
    }

    // TODO: when we stop, save position to blackboard
    @Override
    public void stop() {
        Object endPose = blackboard.getOrDefault(END_POSE_KEY, follower.getPose());
        blackboard.put(END_POSE_KEY, endPose);
    }
}

