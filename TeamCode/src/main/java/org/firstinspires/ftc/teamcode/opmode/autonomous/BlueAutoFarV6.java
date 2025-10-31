package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobotV1;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.SOTM2;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

@Autonomous(name="blue far v6 21 art", group="comp")
public class BlueAutoFarV6 extends OpMode {
    public static final String END_POSE_KEY = "END_POSE";
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobotV1 robot;
    private SOTM2 sotm2;
    // TODO: change the startPose after using the (72, 72) position
    private final Pose startPose = new Pose(54, 6, Math.toRadians(180));
    private final Pose shootPoseStart = new Pose(54, 6, Math.toRadians(180));
    private final Pose shootPoseFar = new Pose(54, 12, Math.toRadians(180));
    private final Pose shootPoseNear = new Pose(62, 74, Math.toRadians(135));

    private final Pose goalPose = new Pose(0, 144, Math.toRadians(45));

    private PathChain intakeFourth, shootFourth, intakeThird, shootThird, intakeSecond, openGate, shootSecond, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeFirst, shootFirst;
    public void buildPaths() {
        intakeFourth = follower.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(54.000, 6.000), new Pose(12.000, 8.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootFourth = follower.pathBuilder()
                .addPath(
                        // Path 2
                        new BezierLine(new Pose(12.000, 8.000), new Pose(54.000, 12.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakeThird = follower.pathBuilder()
                .addPath(
                        // Path 3
                        new BezierCurve(
                                new Pose(54.000, 12.000),
                                new Pose(60.894, 37.915),
                                new Pose(45.383, 37.532),
                                new Pose(13.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootThird = follower.pathBuilder()
                .addPath(
                        // Path 4
                        new BezierLine(new Pose(13.000, 36.000), new Pose(54.000, 12.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakeSecond = follower.pathBuilder()
                .addPath(
                        // Path 5
                        new BezierCurve(
                                new Pose(54.000, 12.000),
                                new Pose(58.213, 63.957),
                                new Pose(48.064, 62.617),
                                new Pose(13.000, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        openGate = follower.pathBuilder()
                .addPath(
                        // Path 6
                        new BezierCurve(
                                new Pose(13.000, 60.000),
                                new Pose(31.213, 58.979),
                                new Pose(31.021, 70.468),
                                new Pose(13.000, 69.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootSecond = follower.pathBuilder()
                .addPath(
                        // Path 7
                        new BezierLine(new Pose(13.000, 69.000), new Pose(62.000, 74.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        intakeGate1 = follower.pathBuilder()
                .addPath(
                        // Path 8
                        new BezierLine(new Pose(62.000, 74.000), new Pose(9.000, 63.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        shootGate1 = follower.pathBuilder()
                .addPath(
                        // Path 9
                        new BezierLine(new Pose(9.000, 63.000), new Pose(62.000, 74.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(
                        // Path 8
                        new BezierLine(new Pose(62.000, 74.000), new Pose(9.000, 63.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        shootGate2 = follower.pathBuilder()
                .addPath(
                        // Path 9
                        new BezierLine(new Pose(9.000, 63.000), new Pose(62.000, 74.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();


        intakeFirst = follower.pathBuilder()
                .addPath(
                        // Path 12
                        new BezierCurve(
                                new Pose(62.000, 74.000),
                                new Pose(61.660, 86.170),
                                new Pose(44.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(
                        // Path 13
                        new BezierLine(new Pose(44.000, 84.000), new Pose(20.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootFirst = follower.pathBuilder()
                .addPath(
                // Path 14
                new BezierLine(new Pose(20.000, 84.000), new Pose(62.000, 74.000))
        )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        robot = new AutonomousRobotV1(hardwareMap);
        sotm2 = new SOTM2(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> {
                            robot.prepareShooting.start();
                        })
                        .transition(new Transition(() -> robot.shooter.atTarget(30))),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeFourth, false);
                            double[] values = sotm2.calculateAzimuthThetaVelocity(shootPoseFar, new Vector());

                            robot.turret.setTarget(values[0]);
                            robot.shooter.setShooterPitch(values[1]);
                            robot.shooter.setTargetVelocity(values[2]);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            follower.followPath(shootFourth, true);
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
                        .onEnter(() -> {
                            follower.followPath(openGate, true);
                            double[] values = sotm2.calculateAzimuthThetaVelocity(shootPoseNear, new Vector());

                            robot.turret.setTarget(values[0]);
                            robot.shooter.setShooterPitch(values[1]);
                            robot.shooter.setTargetVelocity(values[2]);

                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(500), // when there are 9 balls a hard hit will do
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
                            follower.followPath(intakeGate1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
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
                            follower.followPath(intakeGate2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
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
        telemetry.update();
    }

    @Override
    public void start() {
        // constant values. shooter is never turned off after start.
        double[] values = sotm2.calculateAzimuthThetaVelocity(shootPoseStart, new Vector());

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

