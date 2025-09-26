package org.firstinspires.ftc.teamcode.opmode.autonomous;

// this is gonna be a 9+3 auto (9 in classifier, 3 overflow)

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
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.SOTM2;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;
import org.firstinspires.ftc.teamcode.util.pathgenerator.ControlPoint;
import org.firstinspires.ftc.teamcode.util.pathgenerator.PathGenerator;
import org.firstinspires.ftc.teamcode.util.pathgenerator.TargetPose;

@Autonomous(name="blue auto v1")
public class BlueAutoV1 extends OpMode {
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM2 sotm2;
    private final Pose startPose = new Pose(56, 6, Math.toRadians(180));
    private final Pose goalPose = new Pose(12, 132, Math.toRadians(45));
    // actually nvm, i am going to chain drivethird and intakethrid together
    private PathChain shootFirst, intakeSecond, shootSecond, intakeThird, shootThird, intakeFourth, shootFourth, park;

    public void buildPaths() {
        shootFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 6.000), new Pose(60.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakeSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.000, 84.000), new Pose(22.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22.000, 84.000), new Pose(60.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakeThird = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.000, 84.000), new Pose(42.000, 60.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Pose(42.000, 60.000), new Pose(22.000, 60.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootThird = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22.000, 60.000), new Pose(60.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        intakeFourth = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.000, 84.000), new Pose(42.000, 36.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Pose(42.000, 36.000), new Pose(22.000, 36.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootFourth = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22.000, 36.000), new Pose(60.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.000, 84.000), new Pose(60.000, 60.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap);
        sotm2 = new SOTM2(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> follower.followPath(shootFirst, true))
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
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> follower.followPath(shootSecond, true))
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
                        .onEnter(() -> follower.followPath(shootThird, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeFourth, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> follower.followPath(shootFourth, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> follower.followPath(park, true))
                        .transition(new Transition(() -> !follower.isBusy()))
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
    }

    @Override
    public void start() {
        // constant values. shooter is never turned off after start.
        double[] values = sotm2.calculateAzimuthThetaVelocity(startPose, new Vector());

        robot.turret.setTarget(values[0]);
        robot.shooter.setShooterPitch(values[1]);
        robot.shooter.setTargetVelocity(values[2]);
        robot.shooter.setShooterOn(true);

        stateMachine.start();
        robot.start();
    }
}
