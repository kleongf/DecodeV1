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
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobotV1;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.SOTM2;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

@Autonomous(name="21 AURA CLOSE SOTM SO MUCH AURA", group="comp")
public class BlueAutoCloseV7 extends OpMode {
    public static final String END_POSE_KEY = "END_POSE";
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobotV1 robot;
    private SOTM2 sotm2;
    private boolean isSOTMing = true;
    // 14, 66, 227 degrees
    private final Pose startPose = new Pose(30, 137, Math.toRadians(270));
    private Pose shootPose = new Pose(60, 84, Math.toRadians(135));

    private final Pose goalPose = new Pose(0, 144, Math.toRadians(45));

    private PathChain intakeSecond, shootSecond, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeGate3, shootGate3, intakeThird, shootThird, intakeFirst, shootFirst;
    public void buildPaths() {
        intakeSecond = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(30.000, 137.000), new Pose(34.000, 110.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .addPath(
                        new BezierLine(new Pose(34.000, 110.000), new Pose(54.000, 90.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierCurve(
                                new Pose(54.000, 90.000),
                                new Pose(48.064, 54.383),
                                new Pose(32.362, 58.213),
                                new Pose(17.000, 60.000)
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
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                .build();

        intakeGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(51.702, 59.553),
                                new Pose(41.170, 59.553),
                                new Pose(12.9, 61.61)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(142))
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
                .build();

        shootGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(14.000, 63.000),
                                new Pose(30.447, 49.213),
                                new Pose(60.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(142))
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(51.702, 59.553),
                                new Pose(41.170, 59.553),
                                new Pose(12.9, 61.61)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(142))
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
                .build();

        shootGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(14.000, 63.000),
                                new Pose(30.447, 49.213),
                                new Pose(60.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(142))
                .build();

        intakeGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(51.702, 59.553),
                                new Pose(41.170, 59.553),
                                new Pose(12.9, 61.61)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(142))
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
                .build();

        shootGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(14.000, 63.000),
                                new Pose(30.447, 49.213),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(
                        // Path 4
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(51.319, 30.064),
                                new Pose(46.532, 30.255),
                                new Pose(13.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootThird = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(13.000, 36.000), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 84.000), new Pose(20.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.000, 84.000), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                // second
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(0.75);
                            follower.followPath(intakeSecond, false);
                            robot.prepareShooting.start();
                            // robot.prepareIntake.start();
                        })
                        .transition(new Transition(() -> follower.getCurrentPathNumber() == 1)),
                        // .transition(new Transition(() -> !follower.isBusy())),
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

                // gate cycle 1
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            // follower.setMaxPower(0.6);
                            follower.followPath(intakeGate1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(2000),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            ;
                            follower.setMaxPower(1);
                            follower.followPath(shootGate1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // gate cycle 2
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            // follower.setMaxPower(0.6);
                            follower.followPath(intakeGate2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(2000),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            ;
                            follower.setMaxPower(1);
                            follower.followPath(shootGate2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // gate cycle 3

                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            // follower.setMaxPower(0.6);
                            follower.followPath(intakeGate3, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(2000),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            follower.setMaxPower(1);
                            shootPose = new Pose(60, 84, Math.toRadians(180));
                            follower.followPath(shootGate3, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // intake 3

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

                // intake 1

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
        double[] values;
        if (isSOTMing) {
            if (follower.getCurrentPathNumber() < 1) {
                values = sotm2.calculateAzimuthThetaVelocity(new Pose(34, 110, Math.toRadians(180)), new Vector());
            } else {
                values = sotm2.calculateAzimuthThetaVelocity(follower.getPose(), follower.getVelocity());
            }
        } else {
            values = sotm2.calculateAzimuthThetaVelocity(shootPose, new Vector());
        }
        robot.turret.setTarget(values[0]);
        robot.shooter.setShooterPitch(values[1]);
        robot.shooter.setTargetVelocity(values[2]);

        stateMachine.update();
        follower.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        double[] values = sotm2.calculateAzimuthThetaVelocity(new Pose(34, 110, Math.toRadians(180)), new Vector());

        robot.turret.setTarget(values[0]);
        robot.shooter.setShooterPitch(values[1]);
        robot.shooter.setTargetVelocity(values[2]);
        robot.shooter.setShooterOn(true);

        stateMachine.start();
        robot.start();
    }
}
