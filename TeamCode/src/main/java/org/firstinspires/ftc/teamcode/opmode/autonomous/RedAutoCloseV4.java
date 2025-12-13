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
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.SOTM;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

@Autonomous(name="RED 21 AURA CLOSE SOTM optimized and park", group="comp")
public class RedAutoCloseV4 extends OpMode {
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private boolean isSOTMing = true;
    private final Pose startPose = PoseConstants.RED_CLOSE_AUTO_POSE;
    private Pose shootPose = PoseConstants.RED_SHOOT_AUTO_POSE;

    private final Pose goalPose = PoseConstants.RED_GOAL_POSE;
    private PathChain intakeSecond, shootSecond, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeGate3, shootGate3, intakeThird, shootThird, intakeFirst, shootFirst, park;
    public void buildPaths() {
        intakeSecond = follower.pathBuilder()
                .addPath(
                        new BezierLine(PoseConstants.RED_CLOSE_AUTO_POSE, new Pose(144-34.000, 110.000))
                )
                .setLinearHeadingInterpolation(PoseConstants.RED_CLOSE_AUTO_POSE.getHeading(), Math.toRadians(180-144))
                .addPath(
                        new BezierLine(new Pose(144-34.000, 110.000), new Pose(144-54.000, 90.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-144))
                .addPath(
                        new BezierCurve(
                                new Pose(144-54.000, 90.000),
                                new Pose(144-48.064, 54.383),
                                new Pose(144-32.362, 58.213),
                                new Pose(144-15.000, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-144))
                .build();

        shootSecond = follower.pathBuilder()
                .addPath(
                        // Path 2
                        new BezierCurve(
                                new Pose(144-17.000, 60.000),
                                new Pose(144-53.234, 65.681),
                                PoseConstants.RED_SHOOT_AUTO_POSE
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-144), PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .build();

        intakeGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_SHOOT_AUTO_POSE,
                                new Pose(144-49.404, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                new Pose(144-55.340, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .setPathEndTValueConstraint(0.99)
                //.addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
                .build();

        shootGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                PoseConstants.RED_SHOOT_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_SHOOT_AUTO_POSE,
                                new Pose(144-49.404, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                new Pose(144-55.340, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .setPathEndTValueConstraint(0.99)
                //.addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
                .build();

        shootGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                PoseConstants.RED_SHOOT_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .build();

        intakeGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_SHOOT_AUTO_POSE,
                                new Pose(144-49.404, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                new Pose(144-55.340, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                //.addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
                .setPathEndTValueConstraint(0.99)
                .build();

        shootGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                new Pose(144-60,60),
                                new Pose(144-60, 84)
                        )
                )
                .setLinearHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading(), Math.toRadians(180-144))
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-60.000, 84.000), new Pose(144-18.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-144))
                .build();

        shootFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-18.000, 84.000), new Pose(144-60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-144))
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-60, 84),
                                new Pose(144-48.319, 35.064),
                                new Pose(144-44.532, 35.255),
                                new Pose(144-12.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-144))
                .build();

        shootThird = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-13.000, 36.000), new Pose(144-61.5, 28.4))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-144))
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-61.5, 28.4), new Pose(144-61.5, 32))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-144))
                .build();
    }

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap);
        sotm2 = new SOTM(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                // second
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(0.75);
                            follower.followPath(intakeSecond, false);
                            robot.prepareShooting.start();
                        })
                        .transition(new Transition(() -> follower.getCurrentPathNumber() == 1)),
                new State()
                        .onEnter(() -> {
                            robot.startShooting.start();
                            follower.setMaxPower(0.6);
                        })
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            isSOTMing = false;
                            follower.setMaxPower(1);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);
                        })
                        // since the shooting method takes some time let's just wait until path is almost done
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9)),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // gate cycle 1
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeGate1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1250),
                new State()
                        .onEnter(() -> {
                            follower.breakFollowing();
                            follower.setMaxPower(1);
                            follower.followPath(shootGate1, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.8)),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // gate cycle 2
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeGate2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1250),

                new State()
                        .onEnter(() -> {
                            follower.breakFollowing();
                            follower.setMaxPower(1);
                            follower.followPath(shootGate2, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.8)),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // gate cycle 3

                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeGate3, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1250),
                new State()
                        .onEnter(() -> {
                            follower.breakFollowing();
                            follower.setMaxPower(1);
                            shootPose = new Pose(144-60, 84, Math.toRadians(180-144));
                            follower.followPath(shootGate3, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.8)),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // intake 1
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(.8);
                            robot.prepareIntake.start();
                            follower.followPath(intakeFirst, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootFirst, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.85)),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // intake 3
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            shootPose = new Pose(144-61.5, 28.4, Math.toRadians(180-144));
                            follower.followPath(intakeThird, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootThird, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.85)),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShootingFar.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .maxTime(500),//give time for shooter b4 moving

                // park
                new State()
                        .onEnter(() -> follower.followPath(park, true))
                        .onExit(() -> blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose()))
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
        double[] values;
        if (isSOTMing) {
            if (follower.getCurrentPathNumber() < 1) {
                values = sotm2.calculateAzimuthThetaVelocity(new Pose(144-34, 110, Math.toRadians(180-144)), new Vector());
            } else {
                values = sotm2.calculateAzimuthThetaVelocity(follower.getPose(), follower.getVelocity());
                values[0] = sotm2.calculateAzimuthThetaVelocity(new Pose(144-34, 110, Math.toRadians(180-144)), new Vector())[0];
            }
        } else {
            values = sotm2.calculateAzimuthThetaVelocity(shootPose, new Vector());
        }
        robot.setAzimuthThetaVelocity(values);

        stateMachine.update();
        follower.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        double[] values = sotm2.calculateAzimuthThetaVelocity(new Pose(144-34, 110, Math.toRadians(180-144)), new Vector());

        robot.setAzimuthThetaVelocity(values);
        robot.shooter.setShooterOn(true);

        stateMachine.start();
        robot.start();
    }

    @Override
    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
    }
}
