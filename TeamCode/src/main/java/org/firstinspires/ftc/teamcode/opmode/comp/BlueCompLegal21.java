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
import org.firstinspires.ftc.teamcode.util.purepursuit2.MathUtil;

@Autonomous(name="BLUE COMP LEGAL 21", group="?")
public class BlueCompLegal21 extends OpMode {
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private final Pose startPose = PoseConstants.BLUE_FAR_AUTO_POSE;
    private Pose shootPose = new Pose(50, 12, Math.toRadians(180));
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private PathChain shootPreload, intakeSecond, shootSecond, intakeThird, shootThird, intakeCorner, shootCorner, park, openGate, intakePile1, shootPile1, intakePile2, shootPile2, intakePile3, shootPile3;
    public void buildPaths() {
        shootPreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(55.000, 6.000),
                                new Pose(50.000, 12.000)
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        intakeCorner = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.000, 12.000),
                                new Pose(10.000, 10.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootCorner = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.000, 10.000),
                                new Pose(50.000, 12.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeThird = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.000, 12.000),
                                new Pose(51.160, 37.043),
                                new Pose(13.000, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootThird = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.000, 36.000),

                                new Pose(50.000, 12.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeSecond = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.000, 12.000),
                                new Pose(46.700, 60.000),
                                new Pose(13.000, 60.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        openGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(13.000, 60.000),
                                new Pose(30.713, 72.723),
                                new Pose(14.000, 73.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

        shootSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.000, 73.000),

                                new Pose(46, 9)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();

        intakePile1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(10, 9)
                        )
                )
                .setZeroPowerAccelerationMultiplier(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakePile2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(10.000, 9)
                        )
                )
                .setZeroPowerAccelerationMultiplier(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakePile3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(10.000, 9)
                        )
                )
                .setZeroPowerAccelerationMultiplier(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50, 10),
                                new Pose(36, 12)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1)
                .build();

    }

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap, Alliance.BLUE);
        sotm2 = new SOTM(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                // preload
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPreload, true);
                            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
                        })
                        .maxTime(3000) // in case it takes too long
                        .transition(new Transition(() -> robot.shooter.atTarget(20) && !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // corner
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeCorner, false);
                            robot.intakeCommand.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootCorner, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // second
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeSecond, false);
                            robot.intakeCommand.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                // intake third
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            shootPose = new Pose(46, 10, Math.toRadians(180));
                            follower.followPath(intakeThird, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                // open gate
                new State()
                        .onEnter(() -> follower.followPath(openGate, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1500),
                // shoot third
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootThird, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                // pile 1
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile1, false);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.getPose().getX() < 30)),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile1 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, 10),
                                                    new Pose(currentX-4, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(currentX-6, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(10, MathUtil.clamp(currentY+optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setPathEndVelocityConstraint(20)
                                    .setPathEndTValueConstraint(0.9)
                                    .setPathEndHeadingConstraint(Math.toRadians(5))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            shootPile1 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(10, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(46, 10)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile1, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .maxTime(2000)
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile1, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 2
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile2, false);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.getPose().getX() < 30)),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile2 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, 10),
                                                    new Pose(currentX-4, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(currentX-6, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(10, MathUtil.clamp(currentY+optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setPathEndVelocityConstraint(20)
                                    .setPathEndTValueConstraint(0.9)
                                    .setPathEndHeadingConstraint(Math.toRadians(5))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            shootPile2 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(10, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(46, 10)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile2, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .maxTime(2000)
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile2, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 3
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile3, false);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.getPose().getX() < 30)),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile3 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, 10),
                                                    new Pose(currentX-4, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(currentX-6, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(10, MathUtil.clamp(currentY+optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setPathEndVelocityConstraint(20)
                                    .setPathEndTValueConstraint(0.9)
                                    .setPathEndHeadingConstraint(Math.toRadians(5))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            shootPile3 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(10, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(46, 10)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile3, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .maxTime(2000)
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile3, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
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

        follower.setDrivePIDF(new CustomFilteredPIDFCoefficients(0.02,0,0.0006,0.6,0.0));
        follower.setSecondaryDrivePIDF(new CustomFilteredPIDFCoefficients(0.018,0.00,0.0007,0.6,0.0));

        stateMachine.start();
        robot.start();
    }

    @Override
    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
    }
}

