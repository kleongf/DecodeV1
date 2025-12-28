package org.firstinspires.ftc.teamcode.robot.robots;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmode.teleop.Alliance;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.ArtifactVision;
import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.Subsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

import java.util.ArrayList;

public class AutonomousRobot {
    private final ArrayList<Subsystem> subsystems;
    public final BulkRead bulkRead;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;
    public final ArtifactVision vision;

    private final ArrayList<StateMachine> commands;
    public StateMachine intakeCommand;
    public StateMachine shootCommand;
    public StateMachine shootCommandSlow;
    public StateMachine preventMultiPossessionCommand;

    public AutonomousRobot(HardwareMap hardwareMap) {
        subsystems = new ArrayList<>();

        bulkRead = new BulkRead(hardwareMap);
        subsystems.add(bulkRead);

        intake = new Intake(hardwareMap);
        subsystems.add(intake);

        shooter = new Shooter(hardwareMap);
        subsystems.add(shooter);

        turret = new Turret(hardwareMap);
        turret.resetEncoder();
        subsystems.add(turret);

        vision = new ArtifactVision(hardwareMap);
        subsystems.add(vision);

        commands = new ArrayList<>();

        intakeCommand = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                            shooter.closeLatch();
                        })
                        .maxTime(100)
        );
        commands.add(intakeCommand);

        shootCommand = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_OFF;
                            shooter.openLatch();
                        })
                        .maxTime(150),
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                        })
                        // TODO: .transition(new Transition(() -> !intake.intakeFull()))
                        // this does not quite work unless we know exactly how many we have
                        .maxTime(600));
        commands.add(shootCommand);

        shootCommandSlow = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_OFF;
                            shooter.openLatch();
                        })
                        .maxTime(150),
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                        })
                        // TODO: .transition(new Transition(() -> !intake.intakeFull()))
                        // this does not quite work unless we know exactly how many we have
                        .maxTime(1500));
        commands.add(shootCommandSlow);

        // meant to be called _ seconds (usually 0.8?) into a path.
        preventMultiPossessionCommand = new StateMachine(
                new State()
                        .onEnter(() -> intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(200),
                new State()
                        .onEnter(() -> intake.state = Intake.IntakeState.INTAKE_OFF)
                        .maxTime(100)
        );
        commands.add(preventMultiPossessionCommand);
    }

    public void initPositions() {
        // configure shooter
        shooter.state = Shooter.ShooterState.SHOOTER_ON;
        shooter.closeLatch();
        shooter.setTargetVelocity(0);
        shooter.setShooterPitch(Math.toRadians(0));
        // configure turret
        turret.setTarget(0);
        // configure intake
        intake.state = Intake.IntakeState.INTAKE_OFF;
    }

    public void update() {
        for (Subsystem subsystem : subsystems) {
            subsystem.update();
        }
        for (StateMachine command : commands) {
            command.update();
        }
    }

    public void start() {
        for (Subsystem subsystem : subsystems) {
            subsystem.start();
        }
    }

    public void setAzimuthThetaVelocity(double[] values) {
        turret.setTarget(values[0]);
        shooter.setShooterPitch(values[1]);
        shooter.setTargetVelocity(values[2]);
    }

    // TODO: modular autonomous, return states. example
    public StateMachine firstSpikeMark(Alliance alliance, Follower follower, Pose startPose, Pose endPose) {
        // TODO: Make these the same BezierCurves (so it works) and use ConstantHeading so that they will line up fine
        PathChain intake = alliance == Alliance.BLUE ?
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        startPose,
                                        new Pose(55, 84),
                                        new Pose(50, 84),
                                        new Pose(20, 84)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build() :
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        startPose,
                                        new Pose(144 - 55, 84),
                                        new Pose(144 - 50, 84),
                                        new Pose(144 - 20, 84)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();

        PathChain shoot = alliance == Alliance.BLUE ?
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(20, 84),
                                        endPose
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), endPose.getHeading())
                        .build() :
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(144 - 20, 84),
                                        endPose
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), endPose.getHeading())
                        .build();

        return new StateMachine(
                new State()
                        .onEnter(() -> {
                            follower.followPath(intake, true);
                            intakeCommand.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shoot, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> shootCommand.start())
                        .transition(new Transition(() -> shootCommand.isFinished()))
        );
    }

    public StateMachine secondSpikeMark(Alliance alliance, Follower follower, Pose startPose, Pose endPose) {
        // TODO: Make these the same BezierCurves (so it works) and use ConstantHeading so that they will line up fine
        PathChain intake = alliance == Alliance.BLUE ?
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        startPose,
                                        new Pose(55, 60),
                                        new Pose(50, 60),
                                        new Pose(13, 60)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build() :
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        startPose,
                                        new Pose(144 - 55, 60),
                                        new Pose(144 - 50, 60),
                                        new Pose(144 - 13, 60)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();

        PathChain shoot = alliance == Alliance.BLUE ?
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(13, 60),
                                        endPose
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), endPose.getHeading())
                        .build() :
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(144 - 13, 60),
                                        endPose
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), endPose.getHeading())
                        .build();

        return new StateMachine(
                new State()
                        .onEnter(() -> {
                            follower.followPath(intake, true);
                            intakeCommand.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shoot, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> shootCommand.start())
                        .transition(new Transition(() -> shootCommand.isFinished()))
        );
    }

    public StateMachine thirdSpikeMark(Alliance alliance, Follower follower, Pose startPose, Pose endPose) {
        // TODO: Make these the same BezierCurves (so it works) and use ConstantHeading so that they will line up fine
        PathChain intake = alliance == Alliance.BLUE ?
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        startPose,
                                        new Pose(55, 36),
                                        new Pose(50, 36),
                                        new Pose(13, 36)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build() :
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        startPose,
                                        new Pose(144 - 55, 36),
                                        new Pose(144 - 50, 36),
                                        new Pose(144 - 13, 36)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();

        PathChain shoot = alliance == Alliance.BLUE ?
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(13, 36),
                                        endPose
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), endPose.getHeading())
                        .build() :
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(144 - 13, 36),
                                        endPose
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), endPose.getHeading())
                        .build();

        return new StateMachine(
                new State()
                        .onEnter(() -> {
                            follower.followPath(intake, true);
                            intakeCommand.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shoot, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> shootCommand.start())
                        .transition(new Transition(() -> shootCommand.isFinished()))
        );
    }

    public StateMachine preloadClose(Alliance alliance, Follower follower, Pose startPose, Pose endPose) {
        PathChain shoot = alliance == Alliance.BLUE ?
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        startPose,
                                        new Pose(60, 84)
                                )
                        )
                        .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(180))
                        .build() :
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        startPose,
                                        new Pose(144 - 60, 84)
                                )
                        )
                        .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(0))
                        .build();

        return new StateMachine(
                new State()
                        .onEnter(() -> follower.followPath(shoot, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> shootCommand.start())
                        .transition(new Transition(() -> shootCommand.isFinished()))
        );
    }

    // assumes that the robot is facing the direction of the pile. for blue, x neg corresponds to y neg, for red, x neg corresponds to y neg
//    public StateMachine visionPileCycle(Alliance alliance, Follower follower, Pose startPose, Pose endPose) {
//        return new StateMachine(
//                // we have to make the first one a runnable or else it may not work
//                new State()
//                        .onEnter(() -> {
//                            double x = vision.getLargestClusterX();
//                            PathChain intake = alliance == Alliance.BLUE ?
//                                    follower.pathBuilder()
//                                            .addPath(
//                                                    new BezierCurve(
//                                                            startPose,
//                                                            new Pose(50, 24 + x),
//                                                            new Pose(45, 24 + x),
//                                                            new Pose(12, 24 + x)
//                                                    )
//                                            )
//                                            .setConstantHeadingInterpolation(Math.toRadians(180))
//                                            .build() :
//                                    follower.pathBuilder()
//                                            .addPath(
//                                                    new BezierCurve(
//                                                            startPose,
//                                                            new Pose(144 - 50, 24 + x),
//                                                            new Pose(144 - 45, 24 + x),
//                                                            new Pose(144 - 12, 24 + x)
//                                                    )
//                                            )
//                                            .setConstantHeadingInterpolation(Math.toRadians(0))
//                                            .build();
//                            follower.followPath(intake, true);
//                            intakeCommand.start();
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> {
//                            PathChain shoot = alliance == Alliance.BLUE ?
//                                    follower.pathBuilder()
//                                            .addPath(
//                                                    new BezierLine(
//                                                            follower.getPose(),
//                                                            endPose
//                                                    )
//                                            )
//                                            .setLinearHeadingInterpolation(Math.toRadians(180), endPose.getHeading())
//                                            .build() :
//                                    follower.pathBuilder()
//                                            .addPath(
//                                                    new BezierLine(
//                                                            follower.getPose(),
//                                                            endPose
//                                                    )
//                                            )
//                                            .setLinearHeadingInterpolation(Math.toRadians(0), endPose.getHeading())
//                                            .build();
//                            follower.followPath(shoot, true);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> shootCommand.start())
//                        .transition(new Transition(() -> shootCommand.isFinished()))
//        );
//    }

    public StateMachine gateCycle(Alliance alliance, Follower follower, Pose startPose, Pose endPose, double timeAtGate) {
        PathChain intake = alliance == Alliance.BLUE ?
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        startPose,
                                        new Pose(50, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                        new Pose(45, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                        PoseConstants.BLUE_GATE_AUTO_POSE
                                )
                        )
                        .setConstantHeadingInterpolation(PoseConstants.BLUE_GATE_AUTO_POSE.getHeading())
                        .build() :
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        startPose,
                                        new Pose(144 - 50, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                        new Pose(144 - 45, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                        PoseConstants.RED_GATE_AUTO_POSE
                                )
                        )
                        .setConstantHeadingInterpolation(PoseConstants.RED_GATE_AUTO_POSE.getHeading())
                        .build();

        PathChain shoot = alliance == Alliance.BLUE ?
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        PoseConstants.BLUE_GATE_AUTO_POSE,
                                        endPose
                                )
                        )
                        .setLinearHeadingInterpolation(PoseConstants.BLUE_GATE_AUTO_POSE.getHeading(), endPose.getHeading())
                        .build() :
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        PoseConstants.RED_GATE_AUTO_POSE,
                                        endPose
                                )
                        )
                        .setLinearHeadingInterpolation(PoseConstants.RED_GATE_AUTO_POSE.getHeading(), endPose.getHeading())
                        .build();

        return new StateMachine(
                new State()
                        .onEnter(() -> {
                            follower.followPath(intake, true);
                            intakeCommand.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(timeAtGate),
                new State()
                        .onEnter(() -> follower.followPath(shoot, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> shootCommand.start())
                        .transition(new Transition(() -> shootCommand.isFinished()))
        );
    }

    public StateMachine preloadFar(Alliance alliance, Follower follower, Pose startPose, Pose endPose) {
        PathChain shoot = alliance == Alliance.BLUE ?
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        startPose,
                                        new Pose(50, 10)
                                )
                        )
                        .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(180))
                        .build() :
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        startPose,
                                        new Pose(144-50, 10)
                                )
                        )
                        .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(0))
                        .build();

        return new StateMachine(
                new State()
                        .onEnter(() -> follower.followPath(shoot, true))
                        .transition(new Transition(() -> !follower.isBusy() && shooter.atTarget(40))),
                new State()
                        .onEnter(() -> shootCommand.start())
                        .transition(new Transition(() -> shootCommand.isFinished()))
        );
    }

    // TODO: make SOTM preload + intake second modular
}