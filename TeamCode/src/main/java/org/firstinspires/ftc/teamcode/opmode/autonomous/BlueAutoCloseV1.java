package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
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

@Autonomous(name="blue auto close side v1+blackboard")
public class BlueAutoCloseV1 extends OpMode {
    public static final String END_POSE_KEY = "END_POSE";
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM2 sotm2;
    private final Pose startPose = new Pose(30, 138, Math.toRadians(270));
    private final Pose shootPose = new Pose(60, 84, Math.toRadians(180));
    private final Pose goalPose = new Pose(9, 132, Math.toRadians(45));
    private PathChain shootFirst, intakeSecond, shootSecond, intakeThird, openGate, shootThird, intakeFourth, shootFourth, intakeFifth, shootFifth, park;

    public void buildPaths() {
        shootFirst = follower.pathBuilder()
                .addPath(
                        // Path 14
                        new BezierLine(new Pose(30.000, 138.000), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();
        intakeSecond = follower.pathBuilder()
                .addPath(
                        // Path 2
                        new BezierLine(new Pose(60.000, 84.000), new Pose(20.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootSecond = follower.pathBuilder()
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(20.000, 84.000), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakeThird = follower.pathBuilder()
                .addPath(
                        // Path 4
                        new BezierLine(new Pose(60.000, 84.000), new Pose(45.000, 60.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Path 5
                        new BezierLine(new Pose(45.000, 60.000), new Pose(20.000, 60.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        openGate = follower.pathBuilder()
                .addPath(
                        // Path 6
                        new BezierLine(new Pose(20.000, 60.000), new Pose(15.000, 70.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootThird = follower.pathBuilder()
                .addPath(
                        // Path 7
                        new BezierLine(new Pose(15.000, 70.000), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakeFourth = follower.pathBuilder()
                .addPath(
                        // Path 8
                        new BezierLine(new Pose(60.000, 84.000), new Pose(45.000, 36.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Path 9
                        new BezierLine(new Pose(45.000, 36.000), new Pose(20.000, 36.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootFourth = follower.pathBuilder()
                .addPath(
                        // Path 10
                        new BezierLine(new Pose(20.000, 36.000), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakeFifth = follower.pathBuilder()
                .addPath(
                        // Path 11
                        new BezierLine(new Pose(60.000, 84.000), new Pose(9.000, 30.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .addPath(
                        // Path 12
                        new BezierLine(new Pose(9.000, 30.000), new Pose(9.000, 9.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();
        shootFifth = follower.pathBuilder()
                .addPath(
                        // Path 13
                        new BezierLine(new Pose(9.000, 9.000), new Pose(56.000, 8.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();
        park = follower.pathBuilder()
                .addPath(
                        // Path 14
                        new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 40.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();
    }

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap);
        sotm2 = new SOTM2(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> follower.followPath(shootFirst, true))
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
                        .onEnter(() -> follower.followPath(shootSecond, true))
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
                        .onEnter(() -> follower.followPath(openGate, true))
                        .maxTime(2000),
                new State()
                        .onEnter(() -> follower.followPath(shootThird, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeFourth, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> follower.followPath(shootFourth, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeFifth, true);
                            double[] values2 = sotm2.calculateAzimuthThetaVelocity(startPose, new Vector());

                            robot.turret.setTarget(values2[0]);
                            robot.shooter.setShooterPitch(values2[1]);
                            robot.shooter.setTargetVelocity(values2[2]);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> follower.followPath(shootFifth, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> follower.followPath(park, true))
                        .transition(new Transition(() -> !follower.isBusy()))
        );

        Object endPose = blackboard.getOrDefault(END_POSE_KEY, new Pose(56.000, 40.000, Math.toRadians(270)));
        blackboard.put(END_POSE_KEY, endPose);

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
    }

    @Override
    public void start() {
        // constant values. shooter is never turned off after start.
        double[] values = sotm2.calculateAzimuthThetaVelocity(shootPose, new Vector());

        robot.turret.setTarget(values[0]);
        robot.shooter.setShooterPitch(values[1]);
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