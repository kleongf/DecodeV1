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
import org.firstinspires.ftc.teamcode.util.purepursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.util.purepursuit2.PPFollower;
import org.firstinspires.ftc.teamcode.util.purepursuit2.PPPath;

@Autonomous(name="BLUE close 21 pure pursuit", group="not a comp")
public class BlueClose21PurePursuit extends OpMode {
    private PPFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private boolean isSOTMing = true;
    private final Pose startPose = PoseConstants.BLUE_CLOSE_AUTO_POSE;
    private Pose shootPose = PoseConstants.BLUE_SHOOT_AUTO_POSE;

    private double lastTimeStamp = 0;
    private double lastAngleToGoal;
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private PPPath intakeSecond, shootSecond, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeGate3, shootGate3, intakeThird, shootThird, intakeFirst, shootFirst;
    public void buildPaths() {
        intakeSecond = new PPPath(
                PoseConstants.BLUE_CLOSE_AUTO_POSE,
                new Pose(38.000, 115.000, Math.toRadians(180)),
                new Pose(54.000, 90.000, Math.toRadians(180)),
                new Pose(40.064, 60, Math.toRadians(180)),
                new Pose(12.000, 60.000, Math.toRadians(180))
        ).setTangent(false);

        shootSecond = new PPPath(
                new Pose(12.000, 60.000, Math.toRadians(180)),
                new Pose(53.234, 60, PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading()),
                PoseConstants.BLUE_SHOOT_AUTO_POSE
        ).setTangent(false);

        intakeGate1 = new PPPath(
                PoseConstants.BLUE_SHOOT_AUTO_POSE,
                new Pose(49.404, PoseConstants.BLUE_GATE_AUTO_POSE.getY(), PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading()),
                PoseConstants.BLUE_GATE_AUTO_POSE
        ).setTangent(false).setMaxPower(0.8);


        shootGate1 = new PPPath(
                PoseConstants.BLUE_GATE_AUTO_POSE,
                PoseConstants.BLUE_SHOOT_AUTO_POSE
        ).setTangent(false);

        intakeGate2 = new PPPath(
                PoseConstants.BLUE_SHOOT_AUTO_POSE,
                new Pose(49.404, PoseConstants.BLUE_GATE_AUTO_POSE.getY(), PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading()),
                PoseConstants.BLUE_GATE_AUTO_POSE
        ).setTangent(false).setMaxPower(0.8);


        shootGate2 = new PPPath(
                PoseConstants.BLUE_GATE_AUTO_POSE,
                PoseConstants.BLUE_SHOOT_AUTO_POSE
        ).setTangent(false);

        intakeGate3 = new PPPath(
                PoseConstants.BLUE_SHOOT_AUTO_POSE,
                new Pose(49.404, PoseConstants.BLUE_GATE_AUTO_POSE.getY(), PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading()),
                PoseConstants.BLUE_GATE_AUTO_POSE
        ).setTangent(false).setMaxPower(0.8);


        shootGate3 = new PPPath(
                PoseConstants.BLUE_GATE_AUTO_POSE,
                new Pose(60, 84, Math.toRadians(180))
        ).setTangent(false);


        intakeFirst = new PPPath(
                new Pose(60.000, 84.000, Math.toRadians(180)),
                new Pose(17.000, 84.000, Math.toRadians(180))
        ).setTangent(false);


        shootFirst = new PPPath(
                new Pose(18.000, 84.000, Math.toRadians(180)),
                new Pose(60.000, 84.000, Math.toRadians(180))
        ).setTangent(false);

        intakeThird = new PPPath(
                new Pose(60, 84, Math.toRadians(180)),
                new Pose(45.532, 36.000, Math.toRadians(180)),
                new Pose(13.000, 36.000, Math.toRadians(180))
        ).setTangent(false);


        shootThird = new PPPath(
                new Pose(13.000, 36.000),
                new Pose(58, 104)
        );

    }

    @Override
    public void init() {
        follower = new PPFollower(hardwareMap);
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap, Alliance.BLUE);
        sotm2 = new SOTM(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                // second
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeSecond);
                            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
                        })
                        .transition(new Transition(() -> follower.getCurrentPathIndex() == 1)),
                new State()
                        .maxTime(350),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            isSOTMing = false;
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond);
                        })
                        // since the shooting method takes some time let's just wait until path is almost done
                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                // gate cycle 1
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate1);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(800),
                new State()
                        .onEnter(() -> {
                            follower.breakFollowing();
                            follower.followPath(shootGate1);
                        })
                        .maxTime(700),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(200),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                // gate cycle 2
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate2);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(950),
                new State()
                        .onEnter(() -> {
                            follower.breakFollowing();
                            follower.followPath(shootGate2);
                        })
                        .maxTime(700),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(200),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                // gate cycle 3

                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate3);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1300),

                new State()
                        .onEnter(() -> {
                            follower.breakFollowing();
                            follower.followPath(shootGate3);
                            shootPose = new Pose(60, 84, Math.toRadians(180));
                        })
                        .maxTime(700),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(200),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                // intake 1
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeFirst);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootFirst);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                // intake 3
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            // just brute forced it, arctan doesn't work for some reason
                            shootPose = new Pose(58, 104, Math.toRadians(180+56.5)); // Math.toRadians(180)+Math.atan2(104-36, 58-12)
                            follower.followPath(intakeThird);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootThird);
                        })
                        .maxTime(1000),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(200),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> robot.prepareShooting.start())
//                        .transition(new Transition(() -> robot.prepareShooting.isFinished())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .onExit(() -> blackboard.put(RobotConstants.END_POSE_KEY, follower.currentPose))
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
        // this sotm doesn't work, because path starts at idx 0
        // from < 1: set target to pose 38 115
        // from 1 < pathnum < 2: sotm
        if (isSOTMing) {
            if (follower.getCurrentPathIndex() < 1) {
                // this is probably about right, about 30 m/s away from goal. we want shooter vel to change as little as possible.
                values = sotm2.calculateAzimuthThetaVelocity(new Pose(38, 115, Math.toRadians(180)), new Vector(30, Math.toRadians(-45)));
            } else {
                values = sotm2.calculateAzimuthThetaVelocity(follower.currentPose, follower.getCurrentVelocity());
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
        double[] values = sotm2.calculateAzimuthThetaVelocity(new Pose(38, 115, Math.toRadians(180)), new Vector());

        robot.setAzimuthThetaVelocity(values);
        robot.shooter.state = Shooter.ShooterState.SHOOTER_ON;

        stateMachine.start();
        robot.start();
    }

    @Override
    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, follower.currentPose);
    }
}
