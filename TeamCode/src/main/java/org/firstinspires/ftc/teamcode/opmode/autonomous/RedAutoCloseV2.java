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
import org.firstinspires.ftc.teamcode.util.misc.Mirrorer;
import org.firstinspires.ftc.teamcode.util.misc.SOTM2;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

@Autonomous(name="red auto close v2: 18, 2 gate cycles. testing mirrorer")
public class RedAutoCloseV2 extends OpMode {
    public static final String END_POSE_KEY = "END_POSE";
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM2 sotm2;
    private final Pose startPose = new Pose(114, 137, Math.toRadians(270));
    private final Pose shootPoseNear = Mirrorer.mirror(new Pose(60, 84,Math.toRadians(180)));

    private final Pose goalPose = new Pose(144, 144, Math.toRadians(45));

    private PathChain shootPreload, intakeFirst, shootFirst, openGate1, shootGate1, openGate2, shootGate2, intakeSecond, shootSecond, intakeThird, shootThird;
    public void buildPaths() {
        shootPreload = follower.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(114.000, 137.000), Mirrorer.mirror(shootPoseNear))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(
                        // Path 2
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(58.979, 56.872),
                                new Pose(47.872, 59.936),
                                new Pose(10.000, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakeFirst = Mirrorer.mirror(intakeFirst);
        shootFirst = follower.pathBuilder()
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(10.000, 60.000), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        openGate1 = follower.pathBuilder()
                // strafe to gate to make the interpolation more consistent, so that we don't turn into gate.
                .addPath(
                        // Path 4
                        new BezierLine(new Pose(60.000, 84.000), new Pose(30.000, 61.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        // Path 5
                        new BezierLine(new Pose(30.000, 61.000), new Pose(7.000, 61.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();
        openGate1 = Mirrorer.mirror(openGate1);
        shootGate1 = follower.pathBuilder()
                .addPath(
                        // Path 6
                        new BezierLine(new Pose(7.000, 61.000), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
        shootGate1 = Mirrorer.mirror(shootGate1);
        openGate2 = follower.pathBuilder()
                .addPath(
                        // Path 4
                        new BezierLine(new Pose(60.000, 84.000), new Pose(30.000, 61.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        // Path 5
                        new BezierLine(new Pose(30.000, 61.000), new Pose(7.000, 61.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();
        openGate2 = Mirrorer.mirror(openGate2);
        shootGate2 = follower.pathBuilder()
                .addPath(
                        // Path 6
                        new BezierLine(new Pose(7.000, 61.000), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
        shootGate2 = Mirrorer.mirror(shootGate2);
        intakeSecond = follower.pathBuilder()
                .addPath(
                        // Path 9
                        new BezierLine(new Pose(60.000, 84.000), new Pose(12.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakeSecond = Mirrorer.mirror(intakeSecond);
        shootSecond = follower.pathBuilder()
                .addPath(
                        // Path 10
                        new BezierLine(new Pose(12.000, 84.000), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootSecond = Mirrorer.mirror(shootSecond);
        intakeThird = follower.pathBuilder()
                .addPath(
                        // Path 7
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(57.638, 33.511),
                                new Pose(47.681, 35.426),
                                new Pose(10.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakeThird = Mirrorer.mirror(intakeThird);
        shootThird = follower.pathBuilder()
                .addPath(
                        // Path 8
                        new BezierLine(new Pose(10.000, 36.000), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootThird = Mirrorer.mirror(shootThird);
    }

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        follower.setMaxPower(.8);
        robot = new AutonomousRobot(hardwareMap);
        sotm2 = new SOTM2(goalPose);
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
                            follower.followPath(openGate1, true);//opens gate and intakes from same position hopefully
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1750),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            follower.followPath(shootGate1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(openGate2, true);//opens gate and intakes from same position hopefully
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1500),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            follower.followPath(shootGate2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeSecond, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            follower.followPath(shootSecond, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeThird, false);
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
                        .transition(new Transition(() -> robot.startShooting.isFinished()))
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
    }

    @Override
    public void start() {
        // constant values. shooter is never turned off after start.
        double[] values = sotm2.calculateAzimuthThetaVelocity(shootPoseNear, new Vector());

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