package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.FollowerConstants;
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

@Autonomous(name="PATHS ONLY legal blue auto close v5: 18")
public class BlueAutoCloseV5 extends OpMode {
    public static final String END_POSE_KEY = "END_POSE";
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM2 sotm2;
    private final Pose startPose = new Pose(30, 137, Math.toRadians(270));
    private final Pose shootPoseNear = new Pose(60, 84, Math.toRadians(180));
    private final Pose shootPoseFar = new Pose(62, 13, Math.toRadians(180));

    private final Pose goalPose = new Pose(0, 144, Math.toRadians(45));

    private PathChain shootPreload, intakeFirst, shootFirst, intakeThird, shootThird, intakeSecond, openGate, shootSecond, intakeCorner1, shootCorner1, intakeCorner2, shootCorner2;
    public void buildPaths() {
        shootPreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(30.000, 137.000), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();
        intakeFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 84.000), new Pose(20.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();
        shootFirst = follower.pathBuilder()
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(20.000, 84.000), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setPathEndTimeoutConstraint(200)
                .build();
        intakeThird = follower.pathBuilder()
                .addPath(
                        // Path 4
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(57.500, 30.000),
                                new Pose(47.500, 30.000),
                                new Pose(12.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        shootThird = follower.pathBuilder()
                .addPath(
                        // Path 5
                        new BezierCurve(
                                new Pose(12.000, 36.000),
                                new Pose(58.000, 56.000),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setPathEndTimeoutConstraint(200)
                .build();
        intakeSecond = follower.pathBuilder()
                .addPath(
                        // Path 6
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(59.000, 57.000),
                                new Pose(48.000, 60.000),
                                new Pose(13.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        openGate = follower.pathBuilder()
                .addPath(
                        // Path 7
                        new BezierLine(new Pose(13.000, 60.000), new Pose(13.000, 66.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();
        shootSecond = follower.pathBuilder()
                .addPath(
                        // Path 7
                        new BezierLine(new Pose(13.000, 66.000), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .setPathEndTimeoutConstraint(200)
                .build();
        intakeCorner1 = follower.pathBuilder()
                .addPath(
                        // Path 9
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(26.000, 63.000),
                                new Pose(9.000, 58.000),
                                new Pose(7.000, 10.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();
        shootCorner1 = follower.pathBuilder()
                .addPath(
                        // Path 10
                        new BezierCurve(
                                new Pose(7.000, 10.000),
                                new Pose(13.000, 26.000),
                                new Pose(62.000, 13.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .setPathEndTimeoutConstraint(200)
                .build();
        intakeCorner2 = follower.pathBuilder()
                .addPath(
                        // Path 11
                        new BezierLine(new Pose(62.000, 13.000), new Pose(8.000, 11.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        shootCorner2 = follower.pathBuilder()
                .addPath(
                        // Path 12
                        new BezierLine(new Pose(8.000, 11.000), new Pose(62.000, 13.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setPathEndTimeoutConstraint(200)
                .build();
    }

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        // follower.setMaxPower(.8);
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
                        .maxTime(5000),//wait for gate to open fully
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);

                            double[] values2 = sotm2.calculateAzimuthThetaVelocity(new Pose(60, 84, Math.toRadians(270)), new Vector());//the close pose but at 270 degrees

                            robot.turret.setTarget(values2[0]);
                            robot.shooter.setShooterPitch(values2[1]);
                            robot.shooter.setTargetVelocity(values2[2]);
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

                            double[] values2 = sotm2.calculateAzimuthThetaVelocity(shootPoseFar, new Vector());//the close pose but at 270 degrees

                            robot.turret.setTarget(values2[0]);
                            robot.shooter.setShooterPitch(values2[1]);
                            robot.shooter.setTargetVelocity(values2[2]);
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
        robot.shooter.setTargetVelocity(values[2]+50);
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
