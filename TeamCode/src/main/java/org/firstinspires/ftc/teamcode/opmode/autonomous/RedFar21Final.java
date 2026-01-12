package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmode.teleop.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.SOTM;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

@Autonomous(name="RED FAR 21 FINAL", group="not a comp")
public class RedFar21Final extends OpMode {

    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;

    private final Pose startPose =
            new Pose(144 - 52.5, 8.5, Math.toRadians(90));

    private Pose shootPose =
            new Pose(144 - 54, 90, Math.toRadians(290));

    private final Pose goalPose = PoseConstants.RED_GOAL_POSE;

    private PathChain shootPreload, intakeSecond, shootSecond,
            intakeGate1, shootGate1,
            intakeGate2, shootGate2,
            intakeGate3, shootGate3,
            intakeThird, shootThird,
            intakeFirst, shootFirst,
            park;

    public void buildPaths() {

        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        new Pose(144 - 54, 90)
                ))
                .setLinearHeadingInterpolation(
                        PoseConstants.RED_CLOSE_AUTO_POSE.getHeading(),
                        Math.toRadians(290)
                )
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        intakeSecond = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(144 - 54, 90),
                        new Pose(144 - 45, 60),
                        new Pose(144 - 12, 60)
                ))
                .setTangentHeadingInterpolation()
                .build();

        shootSecond = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(144 - 12, 60),
                        new Pose(144 - 30, 50),
                        PoseConstants.RED_SHOOT_AUTO_POSE
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        PoseConstants.RED_SHOOT_AUTO_POSE.getHeading()
                )
                .build();

        intakeGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        PoseConstants.RED_SHOOT_AUTO_POSE,
                        new Pose(144 - 49.404, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                        new Pose(144 - 55.340, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                        PoseConstants.RED_GATE_AUTO_POSE
                ))
                .setConstantHeadingInterpolation(
                        PoseConstants.RED_SHOOT_AUTO_POSE.getHeading()
                )
                .setPathEndTValueConstraint(0.99)
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.8))
                .build();

        shootGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        PoseConstants.RED_GATE_AUTO_POSE,
                        PoseConstants.RED_SHOOT_AUTO_POSE
                ))
                .setConstantHeadingInterpolation(
                        PoseConstants.RED_SHOOT_AUTO_POSE.getHeading()
                )
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        PoseConstants.RED_SHOOT_AUTO_POSE,
                        new Pose(144 - 49.404, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                        new Pose(144 - 55.340, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                        PoseConstants.RED_GATE_AUTO_POSE
                ))
                .setConstantHeadingInterpolation(
                        PoseConstants.RED_SHOOT_AUTO_POSE.getHeading()
                )
                .setPathEndTValueConstraint(0.99)
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.8))
                .build();
        shootGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        PoseConstants.RED_GATE_AUTO_POSE,
                        PoseConstants.RED_SHOOT_AUTO_POSE
                ))
                .setConstantHeadingInterpolation(
                        PoseConstants.RED_SHOOT_AUTO_POSE.getHeading()
                )
                .build();

        intakeGate3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        PoseConstants.RED_SHOOT_AUTO_POSE,
                        new Pose(144 - 49.404, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                        new Pose(144 - 55.340, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                        PoseConstants.RED_GATE_AUTO_POSE
                ))
                .setConstantHeadingInterpolation(
                        PoseConstants.RED_SHOOT_AUTO_POSE.getHeading()
                )
                .setPathEndTValueConstraint(0.99)
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.8))
                .build();

        shootGate3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        PoseConstants.RED_GATE_AUTO_POSE,
                        new Pose(144 - 60, 60),
                        new Pose(144 - 60, 84)
                ))
                .setLinearHeadingInterpolation(
                        PoseConstants.RED_SHOOT_AUTO_POSE.getHeading(),
                        Math.toRadians(0)
                )
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 60, 84),
                        new Pose(144 - 17, 84)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        shootFirst = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 18, 84),
                        new Pose(144 - 60, 84)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(144 - 60, 84),
                        new Pose(144 - 50, 36),
                        new Pose(144 - 12, 36)
                ))
                .setTangentHeadingInterpolation()
                .build();

        shootThird = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 12, 36),
                        new Pose(144 - 50, 12)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 50, 12),
                        new Pose(144 - 36, 12)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        robot = new AutonomousRobot(hardwareMap, Alliance.RED);
        sotm2 = new SOTM(goalPose);

        buildPaths();
        stateMachine = new StateMachine(
                // preload
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPreload, true);
                            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // second
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeSecond, true);
                            robot.intakeCommand.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            shootPose = PoseConstants.RED_SHOOT_AUTO_POSE;
                            follower.followPath(shootSecond, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 1
                new State()
                        .onEnter(() -> {
                            // idea: strengthening translational and increasing drive d will result in higher accuracy + faster braking.

                            robot.intakeCommand.start();
                            follower.followPath(intakeGate1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1000),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate1, true);
                        })
                        .maxTime(700),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(200),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 2
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1100),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate2, true);
                        })
                        .maxTime(700),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(200),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                // gate cycle 3
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate3, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1400),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1); // bruh i ran 0.8 power on that path
                            follower.followPath(shootGate3, true);
                            shootPose = new Pose(144-60, 84, Math.toRadians(180-180));
                        })
                        .maxTime(700),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(200),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // intake 1
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeFirst, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootFirst, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // intake 3
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            shootPose = new Pose(144-50, 12, Math.toRadians(180-180));
                            follower.followPath(intakeThird, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootThird, true);
                        })
                        .maxTime(500),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(200),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                            blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
                        })
                        .onExit(() -> blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose()))
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // park
                new State()
                        .onEnter(() -> {
                            follower.followPath(park, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy()))
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
        double[] values = sotm2.calculateAzimuthThetaVelocity(shootPose, new Vector());
        robot.setAzimuthThetaVelocity(values);

        stateMachine.update();
        follower.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        robot.shooter.state = Shooter.ShooterState.SHOOTER_ON;
        stateMachine.start();
        robot.start();
    }

    @Override
    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
    }
}

