package org.firstinspires.ftc.teamcode.opmode.comp;

import static java.lang.Thread.sleep;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmode.teleop.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.SOTM;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;
import org.firstinspires.ftc.teamcode.util.purepursuit2.MathUtil;

@Autonomous(name="BLUE COMP FAR 27", group="not a comp")
public class BlueCompFar27 extends OpMode {
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private final Pose startPose = new Pose(42.65,9,Math.toRadians(180));
    private Pose shootPose = new Pose(42.65,9,Math.toRadians(180));
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private PathChain intakeCorner, shootCorner, intakeThird, shootThird, intakePile1, shootPile1, intakePile2, shootPile2, intakePile3, shootPile3, intakePile4, shootPile4, intakePile5, shootPile5, intakePile6, shootPile6, intakePile7, shootPile7, park;
    public void buildPaths() {

        intakeCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42.65000, 8.000), new Pose(9.000, 9.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();
        shootCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(9.000, 9.000), new Pose(46,9)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46,9),
                                new Pose(40.000, 36.000),
                                new Pose(38.000, 36.000),
                                new Pose(13.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootThird = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(13.000, 36.000), new Pose(46, 9))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        intakePile1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )
                .setZeroPowerAccelerationMultiplier(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakePile2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )
                .setZeroPowerAccelerationMultiplier(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakePile3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )
                .setZeroPowerAccelerationMultiplier(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakePile4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )
                .setZeroPowerAccelerationMultiplier(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakePile5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )
                .setZeroPowerAccelerationMultiplier(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTValueConstraint(0.9)
                .build();

        intakePile6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )
                .setZeroPowerAccelerationMultiplier(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTValueConstraint(0.9)
                .build();

        intakePile7 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )
                .setZeroPowerAccelerationMultiplier(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(36, 12)
                        )
                )
                .setZeroPowerAccelerationMultiplier(1)
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                        .maxTime(3000) // in case it takes too long
                        .transition(new Transition(() -> robot.shooter.atTarget(20) && !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                            follower.setMaxPower(1);
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // corner
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeCorner, false);
                            shootPose = new Pose(46, 9, Math.toRadians(180));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootCorner, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // third
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeThird, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootThird, true))
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
                                                    new Pose(currentX, 9),
                                                    new Pose(currentX-4, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(currentX-6, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(9, MathUtil.clamp(currentY+optimalX, 9, 40))
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
                                                    new Pose(9, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile1, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
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
                                                    new Pose(currentX, 9),
                                                    new Pose(currentX-4, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(currentX-6, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(9, MathUtil.clamp(currentY+optimalX, 9, 40))
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
                                                    new Pose(9, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile2, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
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
                                                    new Pose(currentX, 9),
                                                    new Pose(currentX-4, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(currentX-6, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(9, MathUtil.clamp(currentY+optimalX, 9, 40))
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
                                                    new Pose(9, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile3, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile3, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 4
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile4, false);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.getPose().getX() < 30)),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile4 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, 9),
                                                    new Pose(currentX-4, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(currentX-6, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(9, MathUtil.clamp(currentY+optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setPathEndVelocityConstraint(20)
                                    .setPathEndTValueConstraint(0.9)
                                    .setPathEndHeadingConstraint(Math.toRadians(5))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            shootPile4 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile4, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile4, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 5
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile5, false);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.getPose().getX() < 30)),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile5 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, 9),
                                                    new Pose(currentX-4, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(currentX-6, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(9, MathUtil.clamp(currentY+optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setPathEndVelocityConstraint(20)
                                    .setPathEndTValueConstraint(0.9)
                                    .setPathEndHeadingConstraint(Math.toRadians(5))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            shootPile5 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile5, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile5, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 6
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile6, false);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.getPose().getX() < 30)),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile6 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, 9),
                                                    new Pose(currentX-4, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(currentX-6, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(9, MathUtil.clamp(currentY+optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setPathEndVelocityConstraint(20)
                                    .setPathEndTValueConstraint(0.9)
                                    .setPathEndHeadingConstraint(Math.toRadians(5))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            shootPile6 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9, MathUtil.clamp(currentY+optimalX, 9, 40)),
                                                    new Pose(46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setZeroPowerAccelerationMultiplier(3)
                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile6, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile6, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(park, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy()))
                        .onExit(() -> blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose())),
                new State()
                        .onEnter(() -> {
                            // just in case
                            blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
                        })
                        .maxTime(100)

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
        robot.turret.setFeedforward(0);
        values = sotm2.calculateAzimuthThetaVelocity(shootPose, new Vector());
        robot.setAzimuthThetaVelocity(values);
        stateMachine.update();
        follower.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        // setting pid coefficients to brake better
        follower.setDrivePIDF(new CustomFilteredPIDFCoefficients(0.015,0,0.0005,0.6,0.0));
        follower.setSecondaryDrivePIDF(new CustomFilteredPIDFCoefficients(0.015,0.00,0.00075,0.6,0.0));
        double[] values = sotm2.calculateAzimuthThetaVelocity(new Pose(42.65, 9, Math.toRadians(180)), new Vector());
        robot.setAzimuthThetaVelocity(values);

        robot.shooter.state = Shooter.ShooterState.SHOOTER_ON;

        stateMachine.start();
        robot.start();
    }

    @Override
    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
    }
}

