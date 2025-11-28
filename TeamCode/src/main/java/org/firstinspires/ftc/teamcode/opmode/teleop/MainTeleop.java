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
import org.firstinspires.ftc.teamcode.robot.subsystems.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.misc.ClosestPoint;
import org.firstinspires.ftc.teamcode.util.misc.SOTM;

import java.util.HashMap;
import java.util.Objects;

public class MainTeleop {
    private Timer localizationTimer;
    private LimelightLocalizer limelightLocalizer;
    private ClosestPoint closestPoint;
    private int state = 0;
    private boolean isAutoDriving = false;
    private Drivetrain drivetrain;
    private double turretOffset = 0;
    private double longitudinalSpeed = 1, lateralSpeed = 1, rotationSpeed = 0.4;
    private TeleopRobot robot;
    // we don't trust blackboard
    // private final Pose startPose = (Pose) blackboard.get(END_POSE_KEY) == null ? new Pose(54, 6, Math.toRadians(180)) : (Pose) blackboard.get(END_POSE_KEY);
    private Pose goalPose;
    private Pose shootPoseFar;
    private Pose gatePose;
    private SmartGamepad gp1;
    private Gamepad gamepad1;
    private SOTM sotm;
    private HashMap<Integer, StateMachine> stateMap;
    private boolean holdingPose = false;

    private double lastTimeStamp = 0;
    private double lastAngleToGoal;
    private Telemetry telemetry;
    private Alliance alliance;

    public MainTeleop(Pose startPose, Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, boolean resetEncoder) {
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setStartingPose(startPose);
        robot = new TeleopRobot(hardwareMap);
        if (resetEncoder) {robot.turret.resetEncoder();}

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        this.alliance = alliance;
        gp1 = new SmartGamepad(gamepad1);

        stateMap = new HashMap<>();
        stateMap.put(0, robot.prepareIntake);
        stateMap.put(1, robot.prepareShooting);
        stateMap.put(2, robot.startShooting);

        sotm = new SOTM(goalPose);
        localizationTimer = new Timer();
        limelightLocalizer = new LimelightLocalizer(hardwareMap);
        limelightLocalizer.start();
        closestPoint = new ClosestPoint();

        // TODO: store these in the PoseConstants stuff
        this.goalPose = alliance == Alliance.BLUE ? PoseConstants.BLUE_GOAL_POSE : PoseConstants.RED_GOAL_POSE;
        this.shootPoseFar = alliance == Alliance.BLUE ? PoseConstants.BLUE_FAR_POSE :  PoseConstants.RED_FAR_POSE;
        this.gatePose = alliance == Alliance.BLUE ? PoseConstants.BLUE_GATE_POSE : PoseConstants.RED_GATE_POSE;
    }

    private double normalizeInput(double input) {
        return Math.signum(input) * Math.sqrt(Math.abs(input));
    }

    public void loop() {
        // we are moving slowly AND its been over 10 seconds
        if (localizationTimer.getElapsedTimeSeconds() > 10 && drivetrain.follower.getVelocity().getMagnitude() < 10) {
            drivetrain.follower.setCurrentPoseWithOffset(limelightLocalizer.update(drivetrain.follower.getPose()));
            localizationTimer.resetTimer();
        }

        gp1.update();

        if (gp1.rightBumperPressed()) {
            state++;
            Objects.requireNonNull(stateMap.get(Math.floorMod(state, 3))).start();
        }

        // x button: drive to gate
        if (gp1.xPressed()) {
            PathChain driveGate = drivetrain.follower.pathBuilder()
                    .addPath(
                            new Path(
                                    new BezierLine(
                                            new Point(drivetrain.follower.getPose()),
                                            new Point((alliance == Alliance.BLUE ? gatePose.getX()+10: gatePose.getX()-10), gatePose.getY())
                                    )
                            )
                    )
                    .setLinearHeadingInterpolation(drivetrain.follower.getPose().getHeading(), gatePose.getHeading())
                    .addPath(
                            new Path(
                                    new BezierLine(
                                            new Point((alliance == Alliance.BLUE ? gatePose.getX()+10: gatePose.getX()-10), gatePose.getY()),
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

        // y button: drive far
        if (gp1.yPressed()) {
            PathChain driveFar = drivetrain.follower.pathBuilder()
                    .addPath(
                            new Path(
                                    new BezierLine(
                                            new Point(drivetrain.follower.getPose()),
                                            new Point(shootPoseFar)
                                    )
                            )
                    )
                    .setLinearHeadingInterpolation(drivetrain.follower.getPose().getHeading(), shootPoseFar.getHeading())
                    .build();
            isAutoDriving = true;
            drivetrain.follower.breakFollowing();
            drivetrain.follower.followPath(driveFar, true);
        }

        // b pressed: closest point
        if (gp1.bPressed()) {
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
            isAutoDriving = true;
            drivetrain.follower.breakFollowing();
            drivetrain.follower.followPath(driveToClosestPoint, true);
        }

        // safety for autodrive
        if (gamepad1.leftStickButtonWasPressed() || gamepad1.rightStickButtonWasPressed()) {
            isAutoDriving = false;
            drivetrain.follower.breakFollowing();
        }

        // relocalization
        if (gp1.dpadUpPressed()) {
            drivetrain.follower.setCurrentPoseWithOffset(new Pose(138, 6, Math.toRadians(90)));
        }

        if (gp1.dpadRightPressed()) {
            turretOffset -= Math.toRadians(2);
        }
        if (gp1.dpadLeftPressed()) {
            turretOffset += Math.toRadians(2);
        }

        if (gamepad1.left_trigger > 0.01 && !holdingPose) {
            holdingPose = true;
            isAutoDriving = true;
            Path holdPointPath = new Path(new BezierPoint(drivetrain.follower.getPose()));
            holdPointPath.setConstantHeadingInterpolation(drivetrain.follower.getPose().getHeading());
            drivetrain.follower.followPath(holdPointPath);
        }

        if (holdingPose && gamepad1.left_trigger < 0.01){
            holdingPose = false;
            isAutoDriving = false;
            drivetrain.follower.breakFollowing();
            drivetrain.setTargetHeading(drivetrain.follower.getPose().getHeading());
        }


        if (!(Math.floorMod(state, 3) == 0)) {
            double[] values = sotm.calculateAzimuthThetaVelocity(drivetrain.follower.getPose(), drivetrain.follower.getVelocity());
            double currentTimeStamp = (double) System.nanoTime() / 1E9;
            if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
            double period = currentTimeStamp - lastTimeStamp;

            double dx = goalPose.getX() - drivetrain.follower.getPose().getX();
            double dy = goalPose.getY() - drivetrain.follower.getPose().getY();
            double currentAngleToGoal = Math.atan2(-dx, dy) - drivetrain.follower.getPose().getHeading() + Math.toRadians(90);
            double vGoal = (currentAngleToGoal-lastAngleToGoal)/period;

            double ff = 0.1 * vGoal;
            robot.turret.setFeedforward(ff);

            lastAngleToGoal = currentAngleToGoal;
            lastTimeStamp = currentTimeStamp;

            robot.turret.setTarget(values[0]+turretOffset);
            robot.shooter.setShooterPitch(values[1]);
            robot.shooter.setTargetVelocity(values[2]);

            telemetry.addData("pitch", values[1]);
            telemetry.addData("velocity", values[2]);
            telemetry.addData("current velocity", robot.shooter.getCurrentVelocity());

        } else {
            double[] values = sotm.calculateAzimuthThetaVelocity(drivetrain.follower.getPose(), drivetrain.follower.getVelocity());
            robot.turret.setTarget(0+turretOffset);
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
            if (Math.abs(gp1.getRightStickX()) > 0) {
                drivetrain.setFieldCentricMovementVectors(normalizeInput(gp1.getLeftStickY()*longitudinalSpeed),
                        normalizeInput(-gp1.getLeftStickX()*lateralSpeed),
                        normalizeInput(gp1.getRightStickX()*rotationSpeed));
            } else {
                drivetrain.setHeadingLockFieldCentricMovementVectors(normalizeInput(gp1.getLeftStickY()*longitudinalSpeed),
                        normalizeInput(-gp1.getLeftStickX()*lateralSpeed),
                        normalizeInput(gp1.getRightStickX()*rotationSpeed));
            }
        }

        telemetry.addData("pose", drivetrain.follower.getPose());
        drivetrain.update();
        robot.update();
        telemetry.update();
    }

    public void start() {
        robot.initPositions();
        robot.shooter.setShooterOn(true);
        robot.start();
    }

    public void stop() {
        Object endPose = drivetrain.follower.getPose();
        blackboard.put(RobotConstants.END_POSE_KEY, endPose);
    }
}
