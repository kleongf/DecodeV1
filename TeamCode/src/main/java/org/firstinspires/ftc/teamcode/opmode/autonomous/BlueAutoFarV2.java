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

@Autonomous(name="blue auto fart side v2")
public class BlueAutoFarV2 extends OpMode {
    public static final String END_POSE_KEY = "END_POSE";
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM2 sotm2;
    private final Pose startPose = new Pose(54, 6, Math.toRadians(90));
    private final Pose shootPose = new Pose(60, 84, Math.toRadians(180));
    private final Pose goalPose = new Pose(0, 144, Math.toRadians(45));
    private PathChain shootFirst, intakeSecond, shootSecond, openGate, shootThird, intakeFourth, shootFourth, intakeFifth, shootFifth, park;

    public void buildPaths() {
        shootFirst = follower.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(54, 6), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
        intakeSecond = follower.pathBuilder()
                .addPath(
                        // Path 2
                        new BezierLine(new Pose(60.000, 84.000), new Pose(45.000, 58.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(45.000, 58.000), new Pose(6.000, 58.000))
                )
                .addParametricCallback(0.1, () -> follower.setMaxPower(0.6))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootSecond = follower.pathBuilder()
                .addPath(
                        // Path 4
                        new BezierCurve(new Pose(6.000, 58.000), new Pose(60, 60), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        openGate = follower.pathBuilder()
                .addPath(
                        // Path 2
                        new BezierLine(new Pose(60.000, 84.000), new Pose(45.000, 57))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(126))
                .addPath(
                        // Path 5
                        new BezierLine(new Pose(45, 57), new Pose(3, 57))
                )
                .setConstantHeadingInterpolation( Math.toRadians(126))
                .build();
        shootThird = follower.pathBuilder()
                .addPath(
                        // Path 6
                        new BezierLine(new Pose(3, 57), new Pose(60, 84))
                )
                .setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(180))
                .build();

        intakeFourth = follower.pathBuilder()
                .addPath(
                        // Path 7
                        new BezierLine(new Pose(60.000, 84.000), new Pose(12.000, 80.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.1, () -> follower.setMaxPower(0.6))
                .build();
        shootFourth = follower.pathBuilder()
                .addPath(
                        // Path 8
                        new BezierLine(new Pose(12.000, 80.000), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakeFifth = follower.pathBuilder()
                .addPath(
                        // Path 9
                        new BezierLine(new Pose(60.000, 84.000), new Pose(45.000, 32.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Path 10
                        new BezierLine(new Pose(45.000, 34.000), new Pose(6, 32.000))
                )
                .addParametricCallback(0.1, () -> follower.setMaxPower(0.6))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootFifth = follower.pathBuilder()
                .addPath(
                        // Path 11
                        new BezierLine(new Pose(6, 32.000), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        park = follower.pathBuilder()
                .addPath(
                        // Path 12
                        new BezierLine(new Pose(60.000, 84.000), new Pose(60.000, 60.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.75);
        robot = new AutonomousRobot(hardwareMap);
        sotm2 = new SOTM2(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> {follower.followPath(shootFirst, true); robot.prepareShooting.start();})
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
                        .onEnter(() -> {robot.prepareShooting.start(); follower.setMaxPower(0.75);})
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
                            follower.followPath(openGate, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(2000),
                new State()
                        .onEnter(() -> {robot.prepareShooting.start(); follower.setMaxPower(0.75);})
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
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
                        .onEnter(() -> {robot.prepareShooting.start(); follower.setMaxPower(0.75);})
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
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {robot.prepareShooting.start(); follower.setMaxPower(0.75);})
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

        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
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

    @Override
    public void stop() {
        Object endPose = blackboard.getOrDefault(END_POSE_KEY, follower.getPose());
        blackboard.put(END_POSE_KEY, endPose);
    }
}