package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
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

@Autonomous(name="21 AURA CLOSE SOTM optimized and park", group="comp")
public class BlueAutoCloseV4 extends OpMode {
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private boolean isSOTMing = true;
    private final Pose startPose = PoseConstants.BLUE_CLOSE_AUTO_POSE;
    private Pose shootPose = PoseConstants.BLUE_SHOOT_AUTO_POSE;

    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private PathChain intakeSecond, shootSecond, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeGate3, shootGate3, intakeThird, shootThird, intakeFirst, shootFirst, park;
    public void buildPaths() {
        intakeSecond = follower.pathBuilder()
                .addPath(
                        new BezierLine(PoseConstants.BLUE_CLOSE_AUTO_POSE, new Pose(34.000, 110.000))
                )
                .setLinearHeadingInterpolation(PoseConstants.BLUE_CLOSE_AUTO_POSE.getHeading(), Math.toRadians(180))
                .addPath(
                        new BezierLine(new Pose(34.000, 110.000), new Pose(54.000, 90.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierCurve(
                                new Pose(54.000, 90.000),
                                new Pose(48.064, 54.383),
                                new Pose(32.362, 58.213),
                                new Pose(15.000, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootSecond = follower.pathBuilder()
                .addPath(
                        // Path 2
                        new BezierCurve(
                                new Pose(17.000, 60.000),
                                new Pose(53.234, 65.681),
                                PoseConstants.BLUE_SHOOT_AUTO_POSE
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading())
                .build();

        intakeGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.BLUE_SHOOT_AUTO_POSE,
                                new Pose(49.404, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                new Pose(55.340, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                PoseConstants.BLUE_GATE_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading())
                .setPathEndTValueConstraint(0.99)
                //.addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
                .build();

        shootGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.BLUE_GATE_AUTO_POSE,
                                PoseConstants.BLUE_SHOOT_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading())
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.BLUE_SHOOT_AUTO_POSE,
                                new Pose(49.404, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                new Pose(55.340, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                PoseConstants.BLUE_GATE_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading())
                .setPathEndTValueConstraint(0.99)
                //.addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
                .build();

        shootGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.BLUE_GATE_AUTO_POSE,
                                PoseConstants.BLUE_SHOOT_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading())
                .build();

        intakeGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.BLUE_SHOOT_AUTO_POSE,
                                new Pose(49.404, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                new Pose(55.340, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                PoseConstants.BLUE_GATE_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading())
                //.addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
                .setPathEndTValueConstraint(0.99)
                .build();

        shootGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.BLUE_GATE_AUTO_POSE,
                                new Pose(60,60),
                                new Pose(60, 84)
                        )
                )
                .setLinearHeadingInterpolation(PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading(), Math.toRadians(180))
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 84.000), new Pose(18.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.000, 84.000), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60, 84),
                                new Pose(48.319, 35.064),
                                new Pose(44.532, 35.255),
                                new Pose(13.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootThird = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(13.000, 36.000), new Pose(61.5, 28.4))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(61.5, 28.4), new Pose(61.5, 32))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                        .maxTime(1500),
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
                        .maxTime(1500),

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
                        .maxTime(1500),
                new State()
                        .onEnter(() -> {
                            follower.breakFollowing();
                            follower.setMaxPower(1);
                            shootPose = new Pose(60, 84, Math.toRadians(180));
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
                            follower.setMaxPower(.6);
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
                            shootPose = new Pose(61.5, 28.4, Math.toRadians(180));
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
                values = sotm2.calculateAzimuthThetaVelocity(new Pose(34, 110, Math.toRadians(180)), new Vector());
            } else {
                values = sotm2.calculateAzimuthThetaVelocity(follower.getPose(), follower.getVelocity());
                values[0] = sotm2.calculateAzimuthThetaVelocity(new Pose(34, 110, Math.toRadians(180)), new Vector())[0];
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
        double[] values = sotm2.calculateAzimuthThetaVelocity(new Pose(34, 110, Math.toRadians(180)), new Vector());

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
