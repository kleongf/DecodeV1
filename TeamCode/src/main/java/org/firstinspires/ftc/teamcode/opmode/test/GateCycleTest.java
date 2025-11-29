//package org.firstinspires.ftc.teamcode.opmode.test;
//
//import static java.lang.Thread.sleep;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Vector;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
//import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
//import org.firstinspires.ftc.teamcode.util.fsm.State;
//import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
//import org.firstinspires.ftc.teamcode.util.fsm.Transition;
//import org.firstinspires.ftc.teamcode.util.misc.SOTM;
//import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;
//
//@Autonomous(name="gate cycle test", group="test")
//public class GateCycleTest extends OpMode {
//    public static final String END_POSE_KEY = "END_POSE";
//    private VoltageCompFollower follower;
//    private StateMachine stateMachine;
//    private AutonomousRobot robot;
//    private SOTM sotm2;
//    // TODO: change the startPose after using the (72, 72) position
//    private final Pose startPose = new Pose(54, 6, Math.toRadians(90));
//    private final Pose shootPose = new Pose(54, 12, Math.toRadians(142));
//
//    private final Pose goalPose = new Pose(0, 144, Math.toRadians(45));
//
//    private PathChain move, intakeFirst, shootFirst, intakeSecond, shootSecond, intakeThird, shootThird, intakeFourth, shootFourth;
//    public void buildPaths() {
//        move = follower.pathBuilder()
//                .addPath(
//                        // Path 1
//                        new BezierLine(new Pose(54.000, 6.000), new Pose(54, 12))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
//                .build();
//
//        intakeFirst = follower.pathBuilder()
//                .addPath(
//                        // Path 1
//                        new BezierLine(new Pose(58.000, 12.000), new Pose(13.5, 61))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(143))
//                .addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
//                .build();
//
//        shootFirst = follower.pathBuilder()
//                .addPath(
//                        // Path 1
//                        new BezierLine(new Pose(13.5, 61), new Pose(58.000, 12.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(143))
//                .build();
//
//        intakeSecond = follower.pathBuilder()
//                .addPath(
//                        // Path 1
//                        new BezierLine(new Pose(58.000, 12.000), new Pose(13.5, 61))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(143))
//                .addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
//                .build();
//
//        shootSecond = follower.pathBuilder()
//                .addPath(
//                        // Path 1
//                        new BezierLine(new Pose(13.5, 61), new Pose(58.000, 12.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(143))
//                .build();
//
//        intakeThird = follower.pathBuilder()
//                .addPath(
//                        // Path 1
//                        new BezierLine(new Pose(58.000, 12.000), new Pose(13.5, 61))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(143))
//                .addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
//                .build();
//
//        shootThird = follower.pathBuilder()
//                .addPath(
//                        // Path 1
//                        new BezierLine(new Pose(13.5, 61), new Pose(58.000, 12.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(143))
//                .build();
//
//
//    }
//
//    @Override
//    public void init() {
//        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
//        follower.setStartingPose(startPose);
//        robot = new AutonomousRobot(hardwareMap);
//        sotm2 = new SOTM(goalPose);
//        buildPaths();
//
//        stateMachine = new StateMachine(
//                new State()
//                        .onEnter(() -> {
//                            follower.followPath(move, true);
//                            robot.prepareIntake.start();
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> {
//                            robot.prepareIntake.start();
//                            follower.followPath(intakeFirst, true);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .maxTime(1000),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
//                new State()
//                        .onEnter(() -> {
//                            robot.slowIntake.start();
//                            ;
//                            follower.followPath(shootFirst, true);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.startShooting.start())
//                        .transition(new Transition(() -> robot.startShooting.isFinished())),
//
//                new State()
//                        .onEnter(() -> {
//                            robot.prepareIntake.start();
//                            follower.followPath(intakeSecond, true);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .maxTime(1000),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
//                new State()
//                        .onEnter(() -> {
//                            robot.slowIntake.start();
//                            ;
//                            follower.followPath(shootSecond, true);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.startShooting.start())
//                        .transition(new Transition(() -> robot.startShooting.isFinished())),
//
//                new State()
//                        .onEnter(() -> {
//                            robot.prepareIntake.start();
//                            follower.followPath(intakeThird, true);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .maxTime(1000),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
//                new State()
//                        .onEnter(() -> {
//                            robot.slowIntake.start();
//                            // ;
//                            follower.followPath(shootThird, true);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.startShooting.start())
//                        .transition(new Transition(() -> robot.startShooting.isFinished()))
//
//
//
//        );
//
//        try {
//            sleep(500);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        robot.initPositions();
//    }
//    @Override
//    public void loop() {
//        stateMachine.update();
//        follower.update();
//        robot.update();
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        follower.setMaxPower(0.75);
//        // constant values. shooter is never turned off after start.
//        double[] values = sotm2.calculateAzimuthThetaVelocity(shootPose, new Vector());
//
//        robot.turret.setTarget(values[0]);
//        robot.shooter.setShooterPitch(values[1]);
//        robot.shooter.setTargetVelocity(values[2]);
//        robot.shooter.setShooterOn(true);
//
//        stateMachine.start();
//        robot.start();
//    }
//
//    // TODO: when we stop, save position to blackboard
//    @Override
//    public void stop() {
//        Object endPose = blackboard.getOrDefault(END_POSE_KEY, follower.getPose());
//        blackboard.put(END_POSE_KEY, endPose);
//    }
//}
