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

@Autonomous(name="blue auto close v2 - 18+, cycle from gate")
public class BlueAutoCloseV2 extends OpMode {
    public static final String END_POSE_KEY = "END_POSE";
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM2 sotm2;
    private final Pose startPose = new Pose(30, 138, Math.toRadians(270));
    private final Pose shootPoseNear = new Pose(49, 85, Math.toRadians(180));
    private final Pose shootPoseFar = new Pose(60, 22, Math.toRadians(180));

    private final Pose goalPose = new Pose(9, 132, Math.toRadians(45));

    private PathChain shootPreload, intakeSecond, shootSecond, openGate1, shootGateCycle1, openGate2, shootGateCycle2, intakeFirst, shootFirst, intakeThird, shootThird;
    public void buildPaths() {
        shootPreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(30, 138), new Pose(49, 85))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();
        intakeSecond = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(49, 85),
                                new Pose(53, 65),
                                new Pose(22, 57)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        shootSecond = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(22, 60),
                                new Pose(53, 65),
                                new Pose(49, 85)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        openGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(49, 85),
                                new Pose(34, 65),
                                new Pose(6, 58.3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                .build();
        shootGateCycle1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(6, 58.3),
                                new Pose(42, 73),
                                new Pose(49, 85)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                .build();
        openGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(49, 85),
                                new Pose(34, 65),
                                new Pose(6, 58.3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                .build();
        shootGateCycle2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(6, 58.3),
                                new Pose(42, 73),
                                new Pose(49, 85)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                .build();
        intakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(49, 85), new Pose(22, 81)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22, 84), new Pose(49, 85)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(49, 85),
                                new Pose(53, 38),
                                new Pose(22, 32)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        shootThird = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22, 36), new Pose(60, 22)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                        .onEnter(() -> follower.followPath(shootPreload, true))
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
                            follower.followPath(openGate1, true);//opens gate and intakes from same position hopefully
                        })
                        .transition(new Transition(() -> !follower.isBusy()))
                        .maxTime(3000)
                        .minTime(2000),//probably can wait a lot less for balls to come out, this is to be safe
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> follower.followPath(shootGateCycle1, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(openGate2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy()))
                        .maxTime(3000)
                        .minTime(2000),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> follower.followPath(shootGateCycle2, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeFirst, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> follower.followPath(shootFirst, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeThird, true);

                            double[] values2 = sotm2.calculateAzimuthThetaVelocity(shootPoseFar, new Vector());

                            robot.turret.setTarget(values2[0]);
                            robot.shooter.setShooterPitch(values2[1]);
                            robot.shooter.setTargetVelocity(values2[2]);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> follower.followPath(shootThird, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished()))
                //no park rn cuz i don't understand rules
        );

        Object endPose = blackboard.getOrDefault(END_POSE_KEY, new Pose(60, 22, Math.toRadians(200)));
        blackboard.put(END_POSE_KEY, endPose);

        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.initPositions();

        follower.setMaxPower(0.75);
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