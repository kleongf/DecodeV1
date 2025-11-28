package org.firstinspires.ftc.teamcode.opmode.test;

import static java.lang.Thread.sleep;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.SOTM;
import org.firstinspires.ftc.teamcode.util.purepursuit.Path2D;
import org.firstinspires.ftc.teamcode.util.purepursuit.Pose2D;
import org.firstinspires.ftc.teamcode.util.purepursuit.PurePursuit;

@Autonomous(name="gate cycle test PURE PURSUIT", group="test")
public class GateCyclePurePursuit extends OpMode {
    private PurePursuit follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    // TODO: change the startPose after using the (72, 72) position
    private final Pose startPose = new Pose(54, 6, Math.toRadians(90));
    private final Pose shootPose = new Pose(54, 12, Math.toRadians(143));

    private final Pose goalPose = new Pose(0, 144, Math.toRadians(45));

    private Path2D move, intakeFirst, shootFirst, intakeSecond, shootSecond, intakeThird, shootThird, intakeFourth, shootFourth;
    public void buildPaths() {
        move = new Path2D(
                new Pose2D(54, 6, Math.toRadians(90)),
                new Pose2D(54, 12, Math.toRadians(143))
        ).setTangent(false).setReversed(false);

        intakeFirst = new Path2D(
                new Pose2D(54, 12, Math.toRadians(143)),
                new Pose2D(15, 61, Math.toRadians(143))
        ).setReversed(false).setTangent(false);

        shootFirst = new Path2D(
                new Pose2D(15, 61, Math.toRadians(143)),
                new Pose2D(54, 12, Math.toRadians(143))
        ).setReversed(false).setTangent(false);

        intakeSecond = new Path2D(
                new Pose2D(54, 12, Math.toRadians(143)),
                new Pose2D(15, 61, Math.toRadians(143))
        ).setReversed(true).setTangent(false);

        shootSecond = new Path2D(
                new Pose2D(15, 61, Math.toRadians(143)),
                new Pose2D(54, 12, Math.toRadians(143))
        ).setReversed(false).setTangent(false);

        intakeThird = new Path2D(
                new Pose2D(54, 12, Math.toRadians(143)),
                new Pose2D(15, 61, Math.toRadians(143))
        ).setReversed(true).setTangent(false);

        shootThird = new Path2D(
                new Pose2D(15, 61, Math.toRadians(143)),
                new Pose2D(54, 12, Math.toRadians(143))
        ).setReversed(true).setTangent(false);

    }



    @Override
    public void init() {
        follower = new PurePursuit(hardwareMap);
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap);
        sotm2 = new SOTM(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> {
                            follower.followPath(move);
                            robot.prepareIntake.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeFirst);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1000),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            ;
                            follower.followPath(shootFirst);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeSecond);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1000),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            ;
                            follower.followPath(shootSecond);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeThird);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1000),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.slowIntake.start();
                            // ;
                            follower.followPath(shootThird);
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
        double[] values = sotm2.calculateAzimuthThetaVelocity(shootPose, new Vector());

        robot.turret.setTarget(values[0]);
        robot.shooter.setShooterPitch(values[1]);
        robot.shooter.setTargetVelocity(values[2]);
        robot.shooter.setShooterOn(true);

        stateMachine.start();
        robot.start();
    }
}

