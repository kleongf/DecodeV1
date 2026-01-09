package org.firstinspires.ftc.teamcode.opmode.teleop;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.util.purepursuit.MathFunctions;

import java.util.HashMap;
import java.util.Objects;

public class MainTeleop {
    public enum RobotState {
        IDLE,
        SHOOTING
    }
    private boolean isHoldingTurret = false;
    private Timer endgameTimer;
    private RobotState robotState;
    private ClosestPoint closestPoint;
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
    private boolean automateRobot = false;
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

        sotm = new SOTM(goalPose);
        // TODO: make different teleops based on far or close. right now it's close.
        // we need a far auto far tele, far auto close tele, close auto close tele, and close auto far tele
        closestPoint = new ClosestPoint(ClosestPoint.ClosestPointType.CLOSE);
        endgameTimer = new Timer();

        this.goalPose = alliance == Alliance.BLUE ? PoseConstants.BLUE_GOAL_POSE : PoseConstants.RED_GOAL_POSE;
        this.parkPose = alliance == Alliance.BLUE ? PoseConstants.BLUE_PARK_POSE :  PoseConstants.RED_PARK_POSE;
        this.gatePose = alliance == Alliance.BLUE ? PoseConstants.BLUE_GATE_POSE : PoseConstants.RED_GATE_POSE;
    }
    // TODO: not sure if Timoe wants this, otherwise i guess Robotcube may still want to be driver
    private double normalizeInput(double input) {
        return 1.2 * Math.signum(input) * Math.sqrt(Math.abs(input));
    }

    private void prepareAutoDrive() {
        drivetrain.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drivetrain.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drivetrain.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drivetrain.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        isAutoDriving = true;
        drivetrain.follower.breakFollowing();
    }

    private void prepareTeleopDrive() {
        drivetrain.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        isAutoDriving = false;
        drivetrain.follower.breakFollowing();
    }

    public void loop() {
        // TODO: a temporary solution
        isHoldingTurret = robotState == RobotState.IDLE;
        // wait: TODO: why do we need intake method? we can just reset intake at the end, so we remove a state?
        // also, why do we even need these states?
        // also for now i will change it so that turret always updates, there's not really a reason for it not to
        // new logic: RobotState is idle or is shooting.
        if (robot.shootCommand.isFinished()) {
            robotState = RobotState.IDLE;
        } else {
            robotState = RobotState.SHOOTING;
        }

        if (automateRobot) {
            // if we are idle and conditions are right, we shoot. realistically this might be kinda stupid, so imma take it out
//            if (
//                    robotState != RobotState.SHOOTING &&
//                            drivetrain.follower.getVelocity().getMagnitude() < 20 &&
//                            robot.shooter.atTarget(20) && // 20 ticks
//                            robot.turret.atTarget(20) && // 20 ticks
//                            robot.intake.intakeFull() &&
//                            robot.inShootingZone(drivetrain.follower.getPose())
//            ) {
//                robot.shootCommand.start();
//            }

            if (!robot.inShootingZone(drivetrain.follower.getPose()) && robot.intake.intakeFull() && !isAutoDriving) {
                // might need to change this to just "push" the robot in the right direction
                PathChain driveToClosestPoint = new PathChain();
                Pose closestPose = closestPoint.closestPose(drivetrain.follower.getPose());
                Pose currentPose = drivetrain.follower.getPose();
                // case 1: the current pose is close to the closestPose, in this case no heading change is best. say it's 20 inches idk
                if (Math.hypot(currentPose.getX()- closestPose.getX(), currentPose.getY()- closestPose.getY()) < 20) {
                    driveToClosestPoint = drivetrain.follower.pathBuilder()
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
                } else {
                    // case 2: the current pose is NOT close to the closestPose, in which case we need to find the closest angle
                    double targetAngle = Math.atan2(closestPose.getY()- currentPose.getY(), closestPose.getX()- currentPose.getX());
                    double currentAngle = currentPose.getHeading();
                    // case 2a: tangential is closer, so we need to turn less
                    if (Math.abs(MathFunctions.angleWrap(targetAngle-currentAngle)) < Math.abs(MathFunctions.angleWrap(targetAngle-currentAngle-Math.PI))) {
                        driveToClosestPoint = drivetrain.follower.pathBuilder()
                                .addPath(
                                        new Path(
                                                new BezierLine(
                                                        new Point(drivetrain.follower.getPose()),
                                                        new Point(closestPoint.closestPose(drivetrain.follower.getPose()))
                                                )
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .build();
                    } else {
                        driveToClosestPoint = drivetrain.follower.pathBuilder()
                                .addPath(
                                        new Path(
                                                new BezierLine(
                                                        new Point(drivetrain.follower.getPose()),
                                                        new Point(closestPoint.closestPose(drivetrain.follower.getPose()))
                                                )
                                        )
                                )
                                .setReversed(true)
                                .setTangentHeadingInterpolation()
                                .build();
                    }
                }
                prepareAutoDrive();
                drivetrain.follower.followPath(driveToClosestPoint, true);
            }
        }

        gp1.update();

        if (gp1.rightBumperPressed()) {
            robot.shootCommand.start();
//            if(Math.hypot(goalPose.getX()-drivetrain.follower.getPose().getX(),goalPose.getY() - drivetrain.follower.getPose().getY())>130){
//                robot.shootCommandSlow.start();
//            }
//            else{
//                robot.shootCommand.start();
//            }
        }

        // slowmo button: turns on/off slowmo, left bumper TODO: uncomment whenever
        if (gp1.leftBumperPressed()) {
            isHoldingTurret = !isHoldingTurret;
        }

//        if (gp1.leftBumperPressed()) {
//            if (longitudinalSpeed == 1 && lateralSpeed == 1 && rotationSpeed == 1) {
//                longitudinalSpeed = 0.5;
//                lateralSpeed = 0.5;
//                rotationSpeed = 0.2;
//            } else {
//                longitudinalSpeed = 1;
//                lateralSpeed = 1;
//                rotationSpeed = 1;
//            }
//        }

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
            prepareAutoDrive();
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
            prepareAutoDrive();
            drivetrain.follower.followPath(park, true);
        }

        // safety for autodrive
        if (gamepad1.leftStickButtonWasPressed() || gamepad1.rightStickButtonWasPressed()) {
            prepareTeleopDrive();
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
            Pose llPose = robot.limelightLocalizer.getCurrentPose(drivetrain.follower.getPose());
            if (llPose.getX() != drivetrain.follower.getPose().getX() && llPose.getY() != drivetrain.follower.getPose().getY()) {
                gamepad1.rumble(300);
                drivetrain.follower.setCurrentPoseWithOffset(llPose);
            }
        }

        if (gp1.dpadRightPressed()) {
            turretOffset -= Math.toRadians(4);
        }
        if (gp1.dpadLeftPressed()) {
            turretOffset += Math.toRadians(4);
        }

        // endgame. can lock until last 20s: endgameTimer.getElapsedTimeSeconds() > 100 if accidentally pressed
        if (gamepad1.left_trigger > 0.01) {
            robot.pivot.setPower(-gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0.01) {
            robot.pivot.setPower(gamepad1.right_trigger);
        } else {
            robot.pivot.setPower(0);
        }



        if (isHoldingTurret) {
            double[] values = sotm.calculateAzimuthThetaVelocity(drivetrain.follower.getPose(), drivetrain.follower.getVelocity());
            robot.turret.setTarget(turretOffset);
            robot.shooter.setShooterPitch(values[1]);
            robot.shooter.setTargetVelocity(values[2]);

            telemetry.addData("pitch", values[1]);
            telemetry.addData("velocity", values[2]);
            telemetry.addData("current velocity", robot.shooter.getCurrentVelocity());
            robot.turret.setFeedforward(0);
        } else {
            double[] values = sotm.calculateAzimuthThetaVelocity(drivetrain.follower.getPose(), drivetrain.follower.getVelocity());

            // robot.turret.setOffset(turretOffset);
            robot.turret.setTarget(values[0]+turretOffset);


            robot.shooter.setShooterPitch(values[1]);
            robot.shooter.setTargetVelocity(values[2]);

            telemetry.addData("pitch", values[1]);
            telemetry.addData("velocity", values[2]);
            telemetry.addData("current velocity", robot.shooter.getCurrentVelocity());
            robot.turret.setFeedforward(0);
        }

        if (isAutoDriving) {
            if (!drivetrain.follower.isBusy()) {
                prepareTeleopDrive();
                drivetrain.setTargetHeading(drivetrain.follower.getPose().getHeading());
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
        prepareTeleopDrive();
    }

    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, drivetrain.follower.getPose());
    }
}
