package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
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
import org.firstinspires.ftc.teamcode.util.purepursuit2.PPFollower;
import org.firstinspires.ftc.teamcode.util.purepursuit2.PPPath;

@Autonomous(name="BLUE CLOSE 21 PP (NEW VERSION!)", group="!")
public class BlueClose21PPNew extends OpMode {
    // this auto can be optimized further by turning on sotm and shooting instantly. all optimizations should be done in this file.
    private PPFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private final Pose startPose = PoseConstants.BLUE_CLOSE_AUTO_POSE;
    private Pose shootPose = new Pose(54, 90, Math.toRadians(-110));
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private PPPath shootPreload, intakeSecond, shootSecond, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeGate3, shootGate3, intakeThird, shootThird, intakeFirst, shootFirst;
    public void buildPaths() {
        shootPreload = new PPPath(
                startPose,
                new Pose(54, 90, Math.toRadians(-110))
        ).setTangent(false);

        intakeSecond = new PPPath(
                new Pose(54, 90, Math.toRadians(-110)),
                new Pose(45, 60),
                new Pose(12, 60)
        ).setPathEndDistanceConstraint(4).setHoldPoint(false).setPathEndSpeedConstraint(15).setPathEndHeadingConstraint(Math.toRadians(5));

        shootSecond = new PPPath(
                new Pose(12.000, 60.000, Math.toRadians(180)),
                new Pose(30, 50, PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading()),
                PoseConstants.BLUE_SHOOT_AUTO_POSE
        ).setTangent(false);

        intakeGate1 = new PPPath(
                PoseConstants.BLUE_SHOOT_AUTO_POSE,
                new Pose(49.404, PoseConstants.BLUE_GATE_AUTO_POSE.getY(), PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading()),
                PoseConstants.BLUE_GATE_AUTO_POSE
        ).setTangent(false).setHoldPointScaleFactor(2).setMaxPower(0.7);


        shootGate1 = new PPPath(
                PoseConstants.BLUE_GATE_AUTO_POSE,
                PoseConstants.BLUE_SHOOT_AUTO_POSE
        ).setTangent(false);

        intakeGate2 = new PPPath(
                PoseConstants.BLUE_SHOOT_AUTO_POSE,
                new Pose(49.404, PoseConstants.BLUE_GATE_AUTO_POSE.getY(), PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading()),
                PoseConstants.BLUE_GATE_AUTO_POSE
        ).setTangent(false).setHoldPointScaleFactor(2).setMaxPower(0.7);


        shootGate2 = new PPPath(
                PoseConstants.BLUE_GATE_AUTO_POSE,
                PoseConstants.BLUE_SHOOT_AUTO_POSE
        ).setTangent(false);

        intakeGate3 = new PPPath(
                PoseConstants.BLUE_SHOOT_AUTO_POSE,
                new Pose(49.404, PoseConstants.BLUE_GATE_AUTO_POSE.getY(), PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading()),
                PoseConstants.BLUE_GATE_AUTO_POSE
        ).setTangent(false).setHoldPointScaleFactor(2).setMaxPower(0.7);


        shootGate3 = new PPPath(
                PoseConstants.BLUE_GATE_AUTO_POSE,
                new Pose(54,60, Math.toRadians(180)),
                new Pose(54, 84, Math.toRadians(180))
        ).setTangent(false);


        intakeFirst = new PPPath(
                new Pose(54, 84, Math.toRadians(180)),
                new Pose(18.000, 84.000, Math.toRadians(180))
        ).setTangent(false).setPathEndDistanceConstraint(4).setHoldPoint(false).setPathEndSpeedConstraint(15).setPathEndHeadingConstraint(Math.toRadians(5));;


        shootFirst = new PPPath(
                new Pose(18.000, 84.000, Math.toRadians(180)),
                new Pose(54, 84, Math.toRadians(180))
        ).setTangent(false);

        intakeThird = new PPPath(
                new Pose(54,84),
                new Pose(50, 36),
                new Pose(12.000, 36.000)
        ).setPathEndDistanceConstraint(4).setHoldPoint(false).setPathEndSpeedConstraint(15).setPathEndHeadingConstraint(Math.toRadians(5));;

        shootThird = new PPPath(
                new Pose(12,36),
                new Pose(54, 115)
        ).setTangent(true).setReversed(true);
    }

    @Override
    public void init() {
        follower = new PPFollower(hardwareMap);
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap, Alliance.BLUE);
        sotm2 = new SOTM(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                // preload
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPreload);
                            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // second
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeSecond);
                            robot.intakeCommand.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            shootPose = PoseConstants.BLUE_SHOOT_AUTO_POSE;
                            follower.followPath(shootSecond);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 1
                new State()
                        .onEnter(() -> {
                            // idea: strengthening translational and increasing drive d will result in higher accuracy + faster braking.

                            robot.intakeCommand.start();
                            follower.followPath(intakeGate1);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(1000),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootGate1);
                        })
                        .maxTime(700),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(200),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),
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
                        .maxTime(1100),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootGate2);
                        })
                        .maxTime(700),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(200),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),
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
                        .maxTime(1400),
                new State()
                        .onEnter(() -> {
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
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // intake 3
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();

                            // just brute forced it, arctan doesn't work for some reason
                            shootPose = new Pose(54, 115, Math.toRadians(-118)); // Math.toRadians(180)+Math.atan2(104-36, 58-12)
                            follower.followPath(intakeThird);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootThird);
                        })
                        .maxTime(700),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(200),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .transition(new Transition(() -> !follower.isBusy())),

                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                            blackboard.put(RobotConstants.END_POSE_KEY, follower.currentPose);
                        })
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
        values = sotm2.calculateAzimuthThetaVelocity(shootPose, new Vector());

        robot.setAzimuthThetaVelocity(values);

        stateMachine.update();
        follower.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        double[] values = sotm2.calculateAzimuthThetaVelocity(shootPose, new Vector());
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
