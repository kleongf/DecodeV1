package org.firstinspires.ftc.teamcode.opmode.comp;

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
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.SOTM;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

@Autonomous(name="RED COMP COMPATIBLE", group="a comp")
public class RedAutoCompatibleComp extends OpMode {
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private final Pose startPose = PoseConstants.RED_CLOSE_AUTO_POSE;
    private Pose shootPose = PoseConstants.RED_SHOOT_AUTO_POSE;
    private double lastTimeStamp = 0;
    private double lastAngleToGoal;
    boolean isSOTMing = true;
    private final Pose goalPose = PoseConstants.RED_GOAL_POSE;
    private PathChain intakeFirst, shootFirst, intakeThird, shootThird, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeSecond, openGate1, openGate2, shootSecond, intakePile1, shootPile1, intakePile2, shootPile2, intakePile3, shootPile3;
    public void buildPaths() {
        intakeSecond = follower.pathBuilder()
                .addPath(
                        new BezierLine(PoseConstants.RED_CLOSE_AUTO_POSE, new Pose(144-38.000, 115.000))
                )
                .setLinearHeadingInterpolation(PoseConstants.RED_CLOSE_AUTO_POSE.getHeading(), Math.toRadians(180-180))
                .addPath(
                        new BezierLine(new Pose(144-38.000, 115.000), new Pose(144-54.000, 90.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-180))
                .addPath(
                        new BezierCurve(
                                new Pose(144-54.000, 90.000),
                                new Pose(144-48.064, 54.383),
                                new Pose(144-32.362, 58.213),
                                new Pose(144-12.000, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();
        
        shootSecond = follower.pathBuilder()
                .addPath(
                        // Path 2
                        new BezierCurve(
                                new Pose(144-17.000, 60.000),
                                new Pose(144-53.234, 65.681),
                                new Pose(144-60,84)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(180-180))
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-60.000, 84.000), new Pose(144-17.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();
        
        openGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-17, 84),
                                new Pose(144-26.5, 82.2),
                                new Pose(144-29, 71.5),
                                new Pose(144-15.000, 73.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(180-270))
                .build();
        
        shootFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-18.000, 84.000), new Pose(144-60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-270), Math.toRadians(180-180))
                .build();
        
        intakeGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-60,84),
                                new Pose(144-49.404, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                new Pose(144-55.340, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .setPathEndTValueConstraint(0.99)
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.8))
                .build();

        shootGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                PoseConstants.RED_SHOOT_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_SHOOT_AUTO_POSE,
                                new Pose(144-49.404, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                new Pose(144-55.340, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setConstantHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading())
                .setPathEndTValueConstraint(0.99)
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.8))
                .build();

        shootGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                PoseConstants.RED_SHOOT_AUTO_POSE
                        )
                )
                .setLinearHeadingInterpolation(PoseConstants.RED_SHOOT_AUTO_POSE.getHeading(), Math.toRadians(180-180))
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                               PoseConstants.RED_SHOOT_AUTO_POSE,
                                new Pose(144-48.319, 35.064),
                                new Pose(144-44.532, 35.255),
                                new Pose(144-11.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();
        
        shootThird = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-13.000, 36.000), new Pose(144-58, 104))
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
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
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(0.8);
                            follower.followPath(intakeSecond, false);
                            robot.prepareShooting.start();
                        })
                        .transition(new Transition(() -> follower.getCurrentPathNumber() == 1)),
                new State()
                        .maxTime(350),
                new State()
                        .onEnter(() -> {
                            robot.startShooting.start();
                            follower.setMaxPower(0.6);
                            follower.setSecondaryDrivePIDF(new CustomFilteredPIDFCoefficients(0.02,0,0.0006,0.6,0.0));
                            follower.setTranslationalPIDF(new CustomPIDFCoefficients(0.07,0,0.006,0.0));
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
                            follower.setSecondaryDrivePIDF(new CustomFilteredPIDFCoefficients(0.02,0,0.0002,0.6,0.0));
                            follower.setTranslationalPIDF(new CustomPIDFCoefficients(0.07,0,0.003,0.0));
                            follower.followPath(shootSecond, true);
                        })
                        // since the shooting method takes some time let's just wait until path is almost done
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.95)),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),
                new State()
                        .onEnter(() -> {
                            isSOTMing = false;
                            follower.setMaxPower(.8);
                            robot.prepareIntake.start();
                            follower.followPath(intakeFirst, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            shootPose = new Pose(144-54, 78, Math.toRadians(180-180));
                            follower.followPath(openGate2, true);
                        })
                        .maxTime(1500),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootFirst, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // gate cycle 1
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeGate1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(900),
                new State()
                        .onEnter(() -> {
                            shootPose = PoseConstants.RED_SHOOT_AUTO_POSE;
                            
                            follower.breakFollowing();
                            follower.setMaxPower(1);
                            follower.followPath(shootGate1, true);
                        })
                        .maxTime(700),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .maxTime(100),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // gate cycle 2
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            follower.followPath(intakeGate2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1050),
                new State()
                        .onEnter(() -> {
                            follower.breakFollowing();
                            follower.setMaxPower(1);
                            follower.followPath(shootGate2, true);
                        })
                        .maxTime(700),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .maxTime(100),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShooting.start())
                        .transition(new Transition(() -> robot.startShooting.isFinished())),

                // intake 3
                new State()
                        .onEnter(() -> {
                            robot.prepareIntake.start();
                            // just brute forced it, arctan doesn't work for some reason
                            shootPose = new Pose(144-58, 104, Math.toRadians(180-180+56.5)); // Math.toRadians(180-180)+Math.atan2(104-36, 58-12)
                            follower.followPath(intakeThird, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.95)),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootThird, true);
                        })
                        .maxTime(700),
                new State()
                        .onEnter(() -> robot.prepareShooting.start())
                        .maxTime(100),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.startShootingFar.start())
                        .onExit(() -> blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose()))
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
            if (follower.getCurrentPathNumber() < 2) {
                // maybe faster updating is better here? idk we can revert to new Vector()
                values = sotm2.calculateAzimuthThetaVelocity(new Pose(144-38, 115, Math.toRadians(180-180)), follower.getVelocity());
                values[2] += 40;
                values[0] += Math.toRadians(3);
                // values[1] -= Math.toRadians(180-0);
                double currentTimeStamp = (double) System.nanoTime() / 1E9;
                if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
                double period = currentTimeStamp - lastTimeStamp;

                double dx = goalPose.getX() - follower.getPose().getX();
                double dy = goalPose.getY() - follower.getPose().getY();
                double currentAngleToGoal = Math.atan2(-dx, dy) - follower.getPose().getHeading() + Math.toRadians(180-90);
                double vGoal = (currentAngleToGoal-lastAngleToGoal)/period;

                double ff = 0.1 * vGoal;
                robot.turret.setFeedforward(ff);
                lastAngleToGoal = currentAngleToGoal;
                lastTimeStamp = currentTimeStamp;
                //values[2] -= 140;
            } else {
                robot.turret.setFeedforward(0);
                values = sotm2.calculateAzimuthThetaVelocity(follower.getPose(), follower.getVelocity());
                // values[0] = sotm2.calculateAzimuthThetaVelocity(new Pose(144-38, 115, Math.toRadians(180-180)), new Vector())[0];
                //values[2] -= 140;
            }
        } else {
            robot.turret.setFeedforward(0);
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
        double[] values = sotm2.calculateAzimuthThetaVelocity(new Pose(144-34, 110, Math.toRadians(180-180)), new Vector());

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
