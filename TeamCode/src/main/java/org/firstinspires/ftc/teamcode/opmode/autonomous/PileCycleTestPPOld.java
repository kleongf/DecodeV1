package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmode.teleop.Alliance;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.SOTM;
import org.firstinspires.ftc.teamcode.util.purepursuit.Path2D;
import org.firstinspires.ftc.teamcode.util.purepursuit.Pose2D;
import org.firstinspires.ftc.teamcode.util.purepursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.util.purepursuit2.PPFollower;

@Autonomous(name="pile cycle test blue 27 PP OLD", group="not a comp")
public class PileCycleTestPPOld extends OpMode {
    private PurePursuit follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private boolean isSOTMing = true;
    private final Pose2D startPose = new Pose2D(42.65,8,Math.toRadians(180));
    private Pose shootPose = new Pose(42.65,8,Math.toRadians(180));
    private final Pose goalPose = new Pose(0, 144, Math.toRadians(45));
    private Path2D intakeCorner, shootCorner, intakeThird, shootThird, intakePile1, shootPile1, intakePile2, shootPile2, intakePile3, shootPile3, intakePile4, shootPile4, intakePile5, shootPile5, intakePile6, shootPile6, intakePile7, shootPile7, intakePile8, shootPile8;
    public void buildPaths() {
        intakeCorner = new Path2D(
                new Pose2D(42.65000, 8.000, Math.toRadians(180)),
                new Pose2D(9.000, 9.000, Math.toRadians(180))
        );

        shootCorner = new Path2D(
                new Pose2D(9.000, 9.000, Math.toRadians(180)),
                new Pose2D(56,20, Math.toRadians(180))
        );

        intakeThird = new Path2D(
                new Pose2D(56,20, Math.toRadians(180)),
                new Pose2D(44.000, 36.000, Math.toRadians(180)),
                new Pose2D(13.000, 36.000, Math.toRadians(180))
        );

        shootThird = new Path2D(
                new Pose2D(13.000, 36.000, Math.toRadians(180)),
                new Pose2D(56, 20, Math.toRadians(180))
        );

    }

    @Override
    public void init() {
        follower = new PurePursuit(hardwareMap);
        follower.setStartingPose(new Pose(startPose.getX(), startPose.getY(), startPose.getHeading()));
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
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // corner
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeCorner);
                            shootPose = new Pose(56, 20, Math.toRadians(180));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootCorner))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // third
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeThird);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootThird))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 1
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            intakePile1 = new Path2D(
                                    new Pose2D(56, 20, Math.toRadians(180)),
                                    new Pose2D(44.000, 20+optimalX, Math.toRadians(180)),
                                    new Pose2D(9.000, 20+optimalX, Math.toRadians(180))
                            );

                            shootPile1 = new Path2D(
                                    new Pose2D(9.000, 20+optimalX, Math.toRadians(180)),
                                    new Pose2D(56, 20, Math.toRadians(180))
                            );
                            robot.intakeCommand.start();
                            follower.followPath(intakePile1);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile1))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 2
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            intakePile2 = new Path2D(
                                    new Pose2D(56, 20, Math.toRadians(180)),
                                    new Pose2D(44.000, 20+optimalX, Math.toRadians(180)),
                                    new Pose2D(9.000, 20+optimalX, Math.toRadians(180))
                            );

                            shootPile2 = new Path2D(
                                    new Pose2D(9.000, 20+optimalX, Math.toRadians(180)),
                                    new Pose2D(56, 20, Math.toRadians(180))
                            );
                            robot.intakeCommand.start();
                            follower.followPath(intakePile2);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile2))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 3
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            intakePile3 = new Path2D(
                                    new Pose2D(56, 20, Math.toRadians(180)),
                                    new Pose2D(44.000, 20+optimalX, Math.toRadians(180)),
                                    new Pose2D(9.000, 20+optimalX, Math.toRadians(180))
                            );

                            shootPile3 = new Path2D(
                                    new Pose2D(9.000, 20+optimalX, Math.toRadians(180)),
                                    new Pose2D(56, 20, Math.toRadians(180))
                            );
                            robot.intakeCommand.start();
                            follower.followPath(intakePile3);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile3))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 4
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            intakePile4 = new Path2D(
                                    new Pose2D(56, 20, Math.toRadians(180)),
                                    new Pose2D(44.000, 20+optimalX, Math.toRadians(180)),
                                    new Pose2D(9.000, 20+optimalX, Math.toRadians(180))
                            );

                            shootPile4 = new Path2D(
                                    new Pose2D(9.000, 20+optimalX, Math.toRadians(180)),
                                    new Pose2D(56, 20, Math.toRadians(180))
                            );
                            robot.intakeCommand.start();
                            follower.followPath(intakePile4);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile4))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 5
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            intakePile5 = new Path2D(
                                    new Pose2D(56, 20, Math.toRadians(180)),
                                    new Pose2D(44.000, 20+optimalX, Math.toRadians(180)),
                                    new Pose2D(9.000, 20+optimalX, Math.toRadians(180))
                            );

                            shootPile5 = new Path2D(
                                    new Pose2D(9.000, 20+optimalX, Math.toRadians(180)),
                                    new Pose2D(56, 20, Math.toRadians(180))
                            );
                            robot.intakeCommand.start();
                            follower.followPath(intakePile5);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile5))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 6
//                new State()
//                        .onEnter(() -> {
//                            double optimalX = robot.vision.getLargestClusterX();
//                            intakePile6 = follower.pathBuilder()
//                                    .addPath(
//                                            new BezierCurve(
//                                                    new Pose2D(56, 20),
//                                                    new Pose2D(49.000, 20+optimalX),
//                                                    new Pose2D(44.000, 20+optimalX),
//                                                    new Pose2D(9.000, 20+optimalX)
//                                            )
//                                    )
//                                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                                    .build();
//                            shootPile6 = follower.pathBuilder()
//                                    .addPath(
//                                            new BezierCurve(
//                                                    new Pose2D(9.000, 20+optimalX),
//                                                    new Pose2D(56, 20)
//                                            )
//                                    )
//                                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                                    .build();
//                            robot.intakeCommand.start();
//                            follower.followPath(intakePile6, false);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> follower.followPath(shootPile6, true))
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.shootCommand.start())
//                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 8
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            intakePile8 = new Path2D(
                                    new Pose2D(56, 20, Math.toRadians(180)),
                                    new Pose2D(44.000, 20+optimalX, Math.toRadians(180)),
                                    new Pose2D(9.000, 20+optimalX, Math.toRadians(180))
                            );

                            shootPile8 = new Path2D(
                                    new Pose2D(9.000, 20+optimalX, Math.toRadians(180)),
                                    new Pose2D(56, 20, Math.toRadians(180))
                            );
                            robot.intakeCommand.start();
                            follower.followPath(intakePile8);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile8))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                            // blackboard.put(RobotConstants.END_Pose_KEY, follower.currentPose2D);
                        })
                        // .onExit(() -> blackboard.put(RobotConstants.END_Pose2D_KEY, follower.currentPose2D))
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
        // System.out.println(robot.vision.currentV);

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
        // blackboard.put(RobotConstants.END_Pose2D_KEY, follower.currentPose2D);
    }
}
