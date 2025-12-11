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
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.SOTM;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

@Autonomous(name="21 3x Blue Pile Cycle", group="comp")
public class BlueAutoCloseV3 extends OpMode {
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private final Pose startPose = PoseConstants.BLUE_CLOSE_AUTO_POSE;
    private Pose shootPose = new Pose(54, 78, Math.toRadians(180));
    boolean isSOTMing = true;
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private PathChain intakeFirst, shootFirst, intakeThird, shootThird, intakeSecond, openGate1, openGate2, shootSecond, intakePile1, shootPile1, intakePile2, shootPile2, intakePile3, shootPile3;
    public void buildPaths() {
        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierLine(PoseConstants.BLUE_CLOSE_AUTO_POSE, new Pose(34.000, 110.000))
                )
                .setLinearHeadingInterpolation(PoseConstants.BLUE_CLOSE_AUTO_POSE.getHeading(), Math.toRadians(180))
                .addPath(
                        new BezierLine(new Pose(34.000, 110.000), new Pose(54.000, 90.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierCurve(
                                new Pose(54.000, 90.000),
                                new Pose(48.064, 54.383),
                                new Pose(50, 34),
                                new Pose(11.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(13.000, 36.000),
                                new Pose(60.000, 40.000),
                                PoseConstants.BLUE_SHOOT_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeSecond = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(60.000, 59.000),
                                new Pose(55.000, 59.000),
                                new Pose(12.000, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        openGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(15.000, 60.000),
                                new Pose(32.362, 59.936),
                                new Pose(32.362, 73.340),
                                new Pose(13.000, 73.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

        shootSecond = follower.pathBuilder()
                .addPath(
                        // Path 2
                        new BezierCurve(
                                new Pose(10, 70),
                                new Pose(53.234, 65.681),
                                PoseConstants.BLUE_SHOOT_AUTO_POSE
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(250))
                .build();
        intakePile1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(PoseConstants.BLUE_SHOOT_AUTO_POSE, new Pose(10, 16))
                )
                .setConstantHeadingInterpolation(Math.toRadians(250))
                .build();

        shootPile1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10, 16), PoseConstants.BLUE_SHOOT_AUTO_POSE)
                )
                .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(180))
                .build();
        intakeFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 84.000), new Pose(18.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        openGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(18, 84),
                                new Pose(26.5, 82.2),
                                new Pose(29, 71.5),
                                new Pose(15.000, 73.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

        shootFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.000, 84.000), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(250))
                .build();


        intakePile2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60, 84), new Pose(10, 16))
                )
                .setConstantHeadingInterpolation(Math.toRadians(250))
                .build();

        shootPile2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10, 16), new Pose(46.000, 10.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(180))
                .build();

        intakePile3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.000, 10.000), new Pose(8, 14))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootPile3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(8, 14), new Pose(46.000, 10.000))
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
                // 3rd spike mark with sotm
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(0.75);
                            follower.followPath(intakeThird, false);
                            robot.prepareShooting.start();
                        })
                        .transition(new Transition(() -> follower.getCurrentPathNumber() == 1)),
                new State()
                        .onEnter(() -> {
                            robot.startShooting.start();
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
                        .onEnter(() -> {
                            follower.followPath(shootThird, true);
                        })
                        // since the shooting method takes some time let's just wait until path is almost done
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9)),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // shoot 1
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(0.8);
                            robot.prepareIntake.start();
                            follower.followPath(intakeSecond, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.99)),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(openGate1, true);
                            shootPose = new Pose(54, 78, Math.toRadians(250));
                        })
                        .maxTime(2000),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9)),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // pile 1
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakePile1, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9)),
                new State()
                        .maxTime(500),
                new State()
                        .onEnter(() -> {
                            shootPose = new Pose(54, 78, Math.toRadians(180));
                            follower.followPath(shootPile1, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9)),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // intake 1
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(0.6);
                            robot.prepareIntake.start();
                            follower.followPath(intakeFirst, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),

                // open gate and wait, no need to wait as long as first ti kme cuz only 3 balls in classifier
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            shootPose = new Pose(54, 78, Math.toRadians(250));
                            follower.followPath(openGate2, true);
                        })
                        .maxTime(1500),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootFirst, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9)),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                // pile 2
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakePile2, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9)),
                new State()
                        .maxTime(750),
                new State()
                        .onEnter(() -> {
                            shootPose = new Pose(46, 10, Math.toRadians(180));
                            follower.followPath(shootPile2, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9)),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // pile 3
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakePile3, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9)),
                new State()
                        .maxTime(500),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile3, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9)),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .onExit(() -> blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose())) // we need this or else no save to blackboard
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
                values[0] = sotm2.calculateAzimuthThetaVelocity(new Pose(34, 110, Math.toRadians(180)), new Vector())[0];
            }
        } else {
            values = sotm2.calculateAzimuthThetaVelocity(shootPose, new Vector());
        }
        robot.setAzimuthThetaVelocity(values);

        stateMachine.update();
        follower.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        double[] values = sotm2.calculateAzimuthThetaVelocity(new Pose(34, 110, Math.toRadians(180)), new Vector());

        robot.setAzimuthThetaVelocity(values);
        robot.shooter.setShooterOn(true);
        stateMachine.start();
        robot.start();
    }

    @Override
    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
    }
}