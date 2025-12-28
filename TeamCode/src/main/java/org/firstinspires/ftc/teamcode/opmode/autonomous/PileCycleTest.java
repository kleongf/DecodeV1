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

@Autonomous(name="pile cycle test blue 30", group="not a comp")
public class PileCycleTest extends OpMode {
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private boolean isSOTMing = true;
    private final Pose startPose = PoseConstants.BLUE_FAR_AUTO_POSE;
    private Pose shootPose = new Pose(48, 9, Math.toRadians(180)); // i dont care but 62, 24, 180 deg
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private PathChain scorePreload, intakeCorner, shootCorner, intakeThird, shootThird, intakePile1, shootPile1, intakePile2, shootPile2, intakePile3, shootPile3, intakePile4, shootPile4, intakePile5, shootPile5, intakePile6, shootPile6, intakePile7, shootPile7;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55.500, 6.000), new Pose(48.000, 9.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        intakeCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(48.000, 9.000), new Pose(9.000, 9.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(9.000, 9.000), new Pose(62.000, 24.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(62.000, 24.000),
                                new Pose(55.000, 36.000),
                                new Pose(50.000, 36.000),
                                new Pose(13.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootThird = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(13.000, 36.000), new Pose(62.000, 24.000))
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
                // preload
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(0.6);
                            follower.followPath(scorePreload, true);
                        })
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
                            follower.followPath(intakeCorner, true);
                            shootPose = new Pose(62, 24, Math.toRadians(180));
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
                            follower.followPath(intakeThird, true);
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
                            double optimalX = robot.vision.getLargestClusterX();
                            intakePile1 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(62.000, 24.000),
                                                    new Pose(55.000, 24+optimalX),
                                                    new Pose(50.000, 24+optimalX),
                                                    new Pose(9.000, 24+optimalX)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            shootPile1 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9.000, 24+optimalX),
                                                    new Pose(62.000, 24.000)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            robot.intakeCommand.start();
                            follower.followPath(intakePile1, true);
                        })
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
                            double optimalX = robot.vision.getLargestClusterX();
                            intakePile2 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(62.000, 24.000),
                                                    new Pose(55.000, 24+optimalX),
                                                    new Pose(50.000, 24+optimalX),
                                                    new Pose(9.000, 24+optimalX)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            shootPile2 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9.000, 24+optimalX),
                                                    new Pose(62.000, 24.000)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            robot.intakeCommand.start();
                            follower.followPath(intakePile2, true);
                        })
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
                            double optimalX = robot.vision.getLargestClusterX();
                            intakePile3 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(62.000, 24.000),
                                                    new Pose(55.000, 24+optimalX),
                                                    new Pose(50.000, 24+optimalX),
                                                    new Pose(9.000, 24+optimalX)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            shootPile3 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9.000, 24+optimalX),
                                                    new Pose(62.000, 24.000)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            robot.intakeCommand.start();
                            follower.followPath(intakePile3, true);
                        })
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
                            double optimalX = robot.vision.getLargestClusterX();
                            intakePile4 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(62.000, 24.000),
                                                    new Pose(55.000, 24+optimalX),
                                                    new Pose(50.000, 24+optimalX),
                                                    new Pose(9.000, 24+optimalX)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            shootPile4 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9.000, 24+optimalX),
                                                    new Pose(62.000, 24.000)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            robot.intakeCommand.start();
                            follower.followPath(intakePile4, true);
                        })
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
                            double optimalX = robot.vision.getLargestClusterX();
                            intakePile5 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(62.000, 24.000),
                                                    new Pose(55.000, 24+optimalX),
                                                    new Pose(50.000, 24+optimalX),
                                                    new Pose(9.000, 24+optimalX)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            shootPile5 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9.000, 24+optimalX),
                                                    new Pose(62.000, 24.000)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            robot.intakeCommand.start();
                            follower.followPath(intakePile5, true);
                        })
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
                            double optimalX = robot.vision.getLargestClusterX();
                            intakePile6 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(62.000, 24.000),
                                                    new Pose(55.000, 24+optimalX),
                                                    new Pose(50.000, 24+optimalX),
                                                    new Pose(9.000, 24+optimalX)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            shootPile6 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9.000, 24+optimalX),
                                                    new Pose(62.000, 24.000)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            robot.intakeCommand.start();
                            follower.followPath(intakePile6, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile6, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 7
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            intakePile5 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(62.000, 24.000),
                                                    new Pose(55.000, 24+optimalX),
                                                    new Pose(50.000, 24+optimalX),
                                                    new Pose(9.000, 24+optimalX)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            shootPile5 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9.000, 24+optimalX),
                                                    new Pose(62.000, 24.000)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            robot.intakeCommand.start();
                            follower.followPath(intakePile5, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile5, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                            blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
                        })
                        .onExit(() -> blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose()))
                        .transition(new Transition(() -> robot.shootCommand.isFinished()))
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
        double[] values = sotm2.calculateAzimuthThetaVelocity(new Pose(48, 9, Math.toRadians(180)), new Vector());
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
