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

@Autonomous(name="BLUE 18 compatible pile", group="a comp")
public class CompatiblePile18 extends OpMode {
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private final Pose startPose = new Pose(42.65,8,Math.toRadians(180));
    private Pose shootPose = new Pose(42.65,8,Math.toRadians(180));

    private double lastTimeStamp = 0;
    private double lastAngleToGoal;
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private PathChain pile1, scorepile1, spike3, scorespike3, pile2, scorepile2, pile3, scorepile3, pile4, scorepile4, pile5, scorepile5, pile6, scorepile6,pile7, scorepile7,pile8, scorepile8;

    public void buildPaths() {
        pile1 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(42.650, 8.000), new Pose(10.000, 8.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorepile1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10.000, 8.000), new Pose(56.000, 20.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        spike3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 20.000),
                                new Pose(55.000, 40.000),
                                new Pose(9.000, 40.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorespike3 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(9.000, 40.000), new Pose(42.650, 8.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pile2 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(42.650, 8.000), new Pose(10.000, 8.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorepile2 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(10.000, 8.000), new Pose(42.650, 8.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pile3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(42.650, 8.000),
                                new Pose(42.650, 25.000),
                                new Pose(10.000, 25.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorepile3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10.000, 25.000), new Pose(42.650, 8.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pile4 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(42.650, 8.000), new Pose(10.000, 8.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorepile4 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(10.000, 8.000), new Pose(42.650, 8.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        pile5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(42.650, 8.000),
                                new Pose(42.650, 25.000),
                                new Pose(10.000, 25.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorepile5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10.000, 25.000), new Pose(42.650, 8.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pile6 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(42.650, 8.000), new Pose(10.000, 8.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorepile6 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(10.000, 8.000), new Pose(42.650, 8.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        pile7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(42.650, 8.000),
                                new Pose(42.650, 25.000),
                                new Pose(10.000, 25.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorepile7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10.000, 25.000), new Pose(42.650, 8.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pile8 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(42.650, 8.000), new Pose(10.000, 8.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorepile8 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(10.000, 8.000), new Pose(42.650, 8.000)))
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
                new State()
                        .onEnter(() -> {
                            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
                        })
                        .maxTime(2000),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(pile1, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(scorepile1, true);
                            shootPose = new Pose(56,20,Math.toRadians(180));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(spike3,false);
                            shootPose = new Pose(42.65,8,Math.toRadians(180));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(scorespike3, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(pile2, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(scorepile2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(pile3, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(scorepile3, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(pile4, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(scorepile4, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(pile5, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(scorepile5, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(pile6, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(scorepile6, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(pile7, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(scorepile7, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(pile8, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(scorepile8, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
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
        double[] values = sotm2.calculateAzimuthThetaVelocity(new Pose(38, 115, Math.toRadians(180)), new Vector());

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
