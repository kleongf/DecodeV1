package org.firstinspires.ftc.teamcode.opmode.comp;

import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
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

@Autonomous(name="RED COMP FAR 21", group="!")
public class RedCompFar21 extends OpMode {
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private final Pose startPose = PoseConstants.RED_FAR_AUTO_POSE;
    private Pose shootPose = new Pose(144-54, 90, Math.toRadians(180-(-110)));
    private final Pose goalPose = PoseConstants.RED_GOAL_POSE;
    private PathChain shootPreload, intakeSecond, shootSecond, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeGate3, shootGate3, intakeThird, shootThird, intakeCorner, shootCorner, park;
    public void buildPaths() {
        shootPreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose, new Pose(144-54, 90))
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(180-(-110)))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        intakeSecond = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-54, 90),
                                // new Pose(144-50, 60),
                                new Pose(144-40, 60),
                                new Pose(144-15, 60)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        shootSecond = follower.pathBuilder()
                .addPath(
                        // Path 2
                        new BezierCurve(
                                new Pose(144-15.000, 60.000),
                                new Pose(144-30, 50),
                                PoseConstants.RED_SHOOT_AUTO_POSE
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-180), PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .build();

        intakeGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_SHOOT_AUTO_POSE,
                                new Pose(144-45.404, PoseConstants.RED_FAR_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_FAR_GATE_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .setPathEndTValueConstraint(0.99)
                .addParametricCallback(0.5, () -> follower.setMaxPower(0.7))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        shootGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_FAR_GATE_AUTO_POSE,
                                PoseConstants.RED_SHOOT_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_SHOOT_AUTO_POSE,
                                new Pose(144-45.404, PoseConstants.RED_FAR_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_FAR_GATE_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .setPathEndTValueConstraint(0.99)
                .addParametricCallback(0.5, () -> follower.setMaxPower(0.7))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        shootGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_FAR_GATE_AUTO_POSE,
                                PoseConstants.RED_SHOOT_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .build();

        intakeGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_SHOOT_AUTO_POSE,
                                new Pose(144-45.404, PoseConstants.RED_FAR_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_FAR_GATE_AUTO_POSE
                                // new Pose(144-55.340, PoseConstants.RED_FAR_GATE_AUTO_POSE.getY()),
                                // new Pose(144-PoseConstants.RED_FAR_GATE_AUTO_POSE.getX(), PoseConstants.RED_FAR_GATE_AUTO_POSE.getY()+1, PoseConstants.RED_FAR_GATE_AUTO_POSE.getHeading())
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .setPathEndTValueConstraint(0.99)
                .addParametricCallback(0.5, () -> follower.setMaxPower(0.7))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        shootGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_FAR_GATE_AUTO_POSE,
                                new Pose(144-50,60),
                                new Pose(144-54, 84)
                        )
                )
                .setLinearHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading(), Math.toRadians(180-180))
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-54.000, 84.000),
                                new Pose(144-45.000, 36.000),
                                new Pose(144-12.000, 36.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        shootThird = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(144-12.000, 36.000),
                                new Pose(144-50, 10)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();

        intakeCorner = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(144-50, 10),
                                new Pose(144-10, 10)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();

        shootCorner = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(144-10, 10),
                                new Pose(144-50, 10)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();



        park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(144-50, 10),
                                new Pose(144-36, 12)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180-180))
                .setZeroPowerAccelerationMultiplier(1)
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
                        .maxTime(300),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                            follower.setHeadingPIDF(new CustomPIDFCoefficients(1,0,0.02,0));
                            follower.setSecondaryHeadingPIDF(new CustomPIDFCoefficients(1.5,0,0.04,0));
                            follower.setSecondaryDrivePIDF(new CustomFilteredPIDFCoefficients(0.02,0,0.0003,0.6,0.0));
                            follower.setDrivePIDF(new CustomFilteredPIDFCoefficients(0.02,0,0.0002,0.6,0.0));
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
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.RED_FAR_GATE_AUTO_POSE_IN), PoseConstants.RED_FAR_GATE_AUTO_POSE_IN.getHeading());
                        })
//                        .minTime(600)
//                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        .maxTime(900),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate1, true);
                        })
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
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.RED_FAR_GATE_AUTO_POSE_IN), PoseConstants.RED_FAR_GATE_AUTO_POSE_IN.getHeading());
                        })
//                        .minTime(600)
//                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        .maxTime(1100),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate2, true);
                        })
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
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.RED_FAR_GATE_AUTO_POSE_IN), PoseConstants.RED_FAR_GATE_AUTO_POSE_IN.getHeading());
                        })
//                        .minTime(600)
//                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        .maxTime(1300),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.setSecondaryDrivePIDF(new CustomFilteredPIDFCoefficients(0.015,0,0.0006,0.6,0.0));
                            follower.followPath(shootGate3, true);
                            shootPose = new Pose(144-54, 84, Math.toRadians(180-180));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // intake 3
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            shootPose = new Pose(144-50, 9, Math.toRadians(180-180));
                            follower.followPath(intakeThird, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootThird, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // intake corner
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeCorner, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootCorner, true);
                        })
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
        double[] values;
        values = sotm2.calculateAzimuthThetaVelocity(shootPose, new Vector());

        robot.setAzimuthThetaVelocity(values);

        stateMachine.update();
        follower.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        double[] values = sotm2.calculateAzimuthThetaVelocity(shootPose, new Vector());
        robot.setAzimuthThetaVelocity(values);
        robot.shooter.state = Shooter.ShooterState.SHOOTER_ON;

        follower.setHeadingPIDF(new CustomPIDFCoefficients(1.5,0,0.03,0));
        follower.setSecondaryHeadingPIDF(new CustomPIDFCoefficients(2,0,0.04,0));
        follower.setDrivePIDF(new CustomFilteredPIDFCoefficients(0.015,0,0.0005,0.6,0.0));
        follower.setSecondaryDrivePIDF(new CustomFilteredPIDFCoefficients(0.015,0.00,0.00075,0.6,0.0));

        stateMachine.start();
        robot.start();
    }

    @Override
    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
    }
}

