package org.firstinspires.ftc.teamcode.opmode.teleop;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.TeleopRobot;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.util.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.misc.ClosestPoint;
import org.firstinspires.ftc.teamcode.util.misc.SOTM;

import java.util.HashMap;
import java.util.Objects;

public class MainTeleop {
    // TODO: SOTM tuning teleop with FTC dash, as well as Pose tuning teleop with dash
    private Timer endgameTimer;
    private ClosestPoint closestPoint;
    private int state = 0;
    private boolean isAutoDriving = false;
    private Drivetrain drivetrain;
    private double turretOffset = 0;
    private double longitudinalSpeed = 1, lateralSpeed = 1, rotationSpeed = 0.4;
    private TeleopRobot robot;
    private Pose goalPose;
    private Pose gatePose;
    private Pose parkPose;
    private SmartGamepad gp1;
    private Gamepad gamepad1;
    public SOTM sotm;
    private HashMap<Integer, StateMachine> stateMap;
    private boolean holdingPose = false;
    private boolean automateRobot = true;
    private Telemetry telemetry;
    private Alliance alliance;

    public MainTeleop(Pose startPose, Pose goalPose, Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, boolean resetEncoder) {
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setStartingPose(startPose);

        robot = new TeleopRobot(hardwareMap);
        if (resetEncoder) {robot.turret.resetEncoder();}

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        this.alliance = alliance;
        gp1 = new SmartGamepad(gamepad1);

        stateMap = new HashMap<>();
        stateMap.put(0, robot.idleCommand);
        stateMap.put(1, robot.shootCommand);

        sotm = new SOTM(goalPose);
        closestPoint = new ClosestPoint();
        endgameTimer = new Timer();

        this.goalPose = alliance == Alliance.BLUE ? PoseConstants.BLUE_GOAL_POSE : PoseConstants.RED_GOAL_POSE;
        this.parkPose = alliance == Alliance.BLUE ? PoseConstants.BLUE_PARK_POSE :  PoseConstants.RED_PARK_POSE;
        this.gatePose = alliance == Alliance.BLUE ? PoseConstants.BLUE_GATE_POSE : PoseConstants.RED_GATE_POSE;
    }
    // TODO: not sure if Timoe wants this, otherwise i guess Robotcube may still want to be driver
    private double normalizeInput(double input) {
        return Math.signum(input) * Math.sqrt(Math.abs(input));
    }

    public void loop() {
        // wait: TODO: why do we need intake method? we can just reset intake at the end, so we remove a state?
        if (automateRobot) {
            // if we are idle and conditions are right, we shoot
            if (
                    drivetrain.follower.getVelocity().getMagnitude() < 10 &&
                    Math.floorMod(state, 2) == 1 &&
                    robot.shooter.atTarget(20) && // 20 ticks
                    robot.turret.atTarget(20) && // 20 ticks
                    robot.intake.intakeFull() &&
                    robot.inShootingZone(drivetrain.follower.getPose())
            ) {
                state++;
                Objects.requireNonNull(stateMap.get(Math.floorMod(state, 2))).start();
            }

            if (
                    !robot.inShootingZone(drivetrain.follower.getPose()) &&
                    robot.intake.intakeFull() &&
                    !isAutoDriving
            ) {
                PathChain driveToClosestPoint = drivetrain.follower.pathBuilder()
                        .addPath(
                                new Path(
                                        new BezierLine(
                                                new Point(drivetrain.follower.getPose()),
                                                new Point(closestPoint.closestPose(drivetrain.follower.getPose()))
                                        )
                                )
                        )
                        .setConstantHeadingInterpolation(drivetrain.follower.getPose().getHeading())
                        .build();
                state++;
                Objects.requireNonNull(stateMap.get(Math.floorMod(state, 2))).start();
                isAutoDriving = true;
                drivetrain.follower.breakFollowing();
                drivetrain.follower.followPath(driveToClosestPoint, true);
            }
        }

        gp1.update();

        if (gp1.rightBumperPressed()) {
            state++;
            Objects.requireNonNull(stateMap.get(Math.floorMod(state, 2))).start();
        }

        // slowmo button: turns on/off slowmo, left bumper
        if (gp1.leftBumperPressed()) {
            if (longitudinalSpeed == 1 && lateralSpeed == 1 && rotationSpeed == 1) {
                longitudinalSpeed = 0.5;
                lateralSpeed = 0.5;
                rotationSpeed = 0.2;
            } else {
                longitudinalSpeed = 1;
                lateralSpeed = 1;
                rotationSpeed = 1;
            }
        }

        // mapped to a button because b button is used for controller
        if (gp1.aPressed()) {
            automateRobot = !automateRobot;
        }

        // x button: drive to gate
        if (gp1.xPressed()) {
            PathChain driveGate = drivetrain.follower.pathBuilder()
                    .addPath(
                            new Path(
                                    new BezierLine(
                                            new Point(drivetrain.follower.getPose()),
                                            new Point((alliance == Alliance.BLUE ? gatePose.getX()+15: gatePose.getX()-15), gatePose.getY())
                                    )
                            )
                    )
                    .setLinearHeadingInterpolation(drivetrain.follower.getPose().getHeading(), gatePose.getHeading())
                    .addPath(
                            new Path(
                                    new BezierLine(
                                            new Point((alliance == Alliance.BLUE ? gatePose.getX()+15: gatePose.getX()-15), gatePose.getY()),
                                            new Point(gatePose)
                                    )
                            )
                    )
                    .setConstantHeadingInterpolation(gatePose.getHeading())
                    .build();
            isAutoDriving = true;
            drivetrain.follower.breakFollowing();
            drivetrain.follower.followPath(driveGate, true);
        }

        // y button: park
        if (gp1.yPressed()) {
            PathChain park = drivetrain.follower.pathBuilder()
                    .addPath(
                            new Path(
                                    new BezierLine(
                                            new Point(drivetrain.follower.getPose()),
                                            new Point(parkPose)
                                    )
                            )
                    )
                    .setLinearHeadingInterpolation(drivetrain.follower.getPose().getHeading(), parkPose.getHeading())
                    .build();
            isAutoDriving = true;
            drivetrain.follower.breakFollowing();
            drivetrain.follower.followPath(park, true);
        }

        // safety for autodrive
        if (gamepad1.leftStickButtonWasPressed() || gamepad1.rightStickButtonWasPressed()) {
            isAutoDriving = false;
            drivetrain.follower.breakFollowing();
        }

        // relocalization
        if (gp1.dpadUpPressed()) {
            if (alliance == Alliance.BLUE) {
                drivetrain.follower.setCurrentPoseWithOffset(new Pose(136.5, 6, Math.toRadians(90)));
            } else {
                drivetrain.follower.setCurrentPoseWithOffset(new Pose(7.5, 6, Math.toRadians(90)));
            }
        }

        if (gp1.dpadDownPressed()) {
            Pose llPose = robot.vision.getCurrentPose(drivetrain.follower.getPose());
            if (llPose.getX() != drivetrain.follower.getPose().getX() && llPose.getY() != drivetrain.follower.getPose().getY()) {
                gamepad1.rumble(300);
                drivetrain.follower.setCurrentPoseWithOffset(llPose);
            }
        }

        if (gp1.dpadRightPressed()) {
            turretOffset -= Math.toRadians(2);
        }
        if (gp1.dpadLeftPressed()) {
            turretOffset += Math.toRadians(2);
        }

        // endgame. can lock until last 20s: endgameTimer.getElapsedTimeSeconds() > 100 if accidentally pressed
        if (gamepad1.left_trigger > 0.01) {
            robot.pivot.setPower(-gamepad1.left_trigger);
        }

        if (gamepad1.right_trigger > 0.01) {
            robot.pivot.setPower(gamepad1.left_trigger);
        }

        if (!(Math.floorMod(state, 3) == 0)) {
            // working on new sotm don't need this
            double[] values = sotm.calculateAzimuthThetaVelocity(drivetrain.follower.getPose(), drivetrain.follower.getVelocity());
            robot.turret.setTarget(values[0]+turretOffset);
            robot.shooter.setShooterPitch(values[1]);
            robot.shooter.setTargetVelocity(values[2]);

            telemetry.addData("pitch", values[1]);
            telemetry.addData("velocity", values[2]);
            telemetry.addData("current velocity", robot.shooter.getCurrentVelocity());

        } else {
            double[] values = sotm.calculateAzimuthThetaVelocity(drivetrain.follower.getPose(), drivetrain.follower.getVelocity());
            robot.turret.setTarget(0 + turretOffset);
            robot.shooter.setShooterPitch(values[1]);
            robot.shooter.setTargetVelocity(values[2]);

            telemetry.addData("pitch", values[1]);
            telemetry.addData("velocity", values[2]);
            telemetry.addData("current velocity", robot.shooter.getCurrentVelocity());
            robot.turret.setFeedforward(0);
        }

        if (isAutoDriving) {
            if (!holdingPose) {
                if (!drivetrain.follower.isBusy()) {
                    isAutoDriving = false;
                    drivetrain.follower.breakFollowing();
                    drivetrain.setTargetHeading(drivetrain.follower.getPose().getHeading());
                }
            }

        } else {
            if (alliance == Alliance.BLUE) {
                if (Math.abs(gp1.getRightStickX()) > 0) {
                    drivetrain.setFieldCentricMovementVectors(normalizeInput(gp1.getLeftStickY()*longitudinalSpeed),
                            normalizeInput(-gp1.getLeftStickX()*lateralSpeed),
                            normalizeInput(gp1.getRightStickX()*rotationSpeed));
                } else {
                    drivetrain.setHeadingLockFieldCentricMovementVectors(normalizeInput(gp1.getLeftStickY()*longitudinalSpeed),
                            normalizeInput(-gp1.getLeftStickX()*lateralSpeed),
                            normalizeInput(gp1.getRightStickX()*rotationSpeed));
                }
            } else if (alliance == Alliance.RED) {
                if (Math.abs(gp1.getRightStickX()) > 0) {
                    drivetrain.setFieldCentricMovementVectors(normalizeInput(-gp1.getLeftStickY()*longitudinalSpeed),
                            normalizeInput(gp1.getLeftStickX()*lateralSpeed),
                            normalizeInput(gp1.getRightStickX()*rotationSpeed));
                } else {
                    drivetrain.setHeadingLockFieldCentricMovementVectors(normalizeInput(-gp1.getLeftStickY()*longitudinalSpeed),
                            normalizeInput(gp1.getLeftStickX()*lateralSpeed),
                            normalizeInput(gp1.getRightStickX()*rotationSpeed));
                }
            }

        }

        telemetry.addData("pose", drivetrain.follower.getPose());
        drivetrain.update();
        robot.update();
        telemetry.update();
    }


    public void start() {
        robot.initPositions();
        robot.start();
        endgameTimer.resetTimer();
    }

    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, drivetrain.follower.getPose());
    }
}
