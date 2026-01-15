package org.firstinspires.ftc.teamcode.opmode.teleop;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.TeleopRobot;
import org.firstinspires.ftc.teamcode.util.hardware.SimplePath;
import org.firstinspires.ftc.teamcode.util.hardware.SimplePathChain;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.hardware.SpecializedDrivetrain;
import org.firstinspires.ftc.teamcode.util.misc.ClosestPoint;
import org.firstinspires.ftc.teamcode.util.misc.SOTM2;
import org.firstinspires.ftc.teamcode.util.purepursuit.MathFunctions;


public class MainTeleopV2 {
    public enum RobotState {
        IDLE,
        SHOOTING
    }
    private boolean isHoldingTurret = false;
    private RobotState robotState;
    private ClosestPoint closestPoint;
    private boolean isAutoDriving = false;
    private SpecializedDrivetrain drivetrain;
    private double turretOffset = 0;
    private double longitudinalSpeed = 1, lateralSpeed = 1, rotationSpeed = 0.4;
    private TeleopRobot robot;
    private Pose gatePose;
    private Pose parkPose;
    private SmartGamepad gp1;
    private Gamepad gamepad1;
    public SOTM2 sotm;
    private boolean automateRobot = true;
    private Telemetry telemetry;
    private Alliance alliance;

    public MainTeleopV2(Pose startPose, Pose goalPose, Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, boolean resetEncoder) {
        drivetrain = new SpecializedDrivetrain(hardwareMap);
        drivetrain.setStartingPose(startPose);

        robot = new TeleopRobot(hardwareMap);
        if (resetEncoder) {robot.turret.resetEncoder();}

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        this.alliance = alliance;
        gp1 = new SmartGamepad(gamepad1);

        sotm = new SOTM2(goalPose);
        closestPoint = new ClosestPoint(ClosestPoint.ClosestPointType.CLOSE);

        this.parkPose = alliance == Alliance.BLUE ? PoseConstants.BLUE_PARK_POSE :  PoseConstants.RED_PARK_POSE;
        this.gatePose = alliance == Alliance.BLUE ? PoseConstants.BLUE_GATE_POSE : PoseConstants.RED_GATE_POSE;
    }
    private double normalizeInput(double input) {
        return 1.2 * Math.signum(input) * Math.sqrt(Math.abs(input));
    }

    public void loop() {
        // TODO: a temporary solution
        // isHoldingTurret = robotState == RobotState.IDLE;
        isHoldingTurret = false;

        if (robot.shootCommand.isFinished()) {
            robotState = RobotState.IDLE;
        } else {
            robotState = RobotState.SHOOTING;
        }

        if (automateRobot) {
            if (!robot.inShootingZone(drivetrain.follower.getPose()) && robot.intake.intakeFull() && !isAutoDriving && robotState == RobotState.IDLE) {

                SimplePathChain kickToClosest;
                Pose closestPose = closestPoint.closestPose(drivetrain.follower.getPose());
                Pose currentPose = drivetrain.follower.getPose();
                // case 1: the current pose is close to the closestPose, in this case no heading change is best. say it's 20 inches idk
                if (Math.hypot(currentPose.getX()- closestPose.getX(), currentPose.getY()- closestPose.getY()) < 20) {
                    kickToClosest = new SimplePathChain(
                            new SimplePath(currentPose, new Pose(closestPose.getX(), closestPose.getY(), currentPose.getHeading()))
                    );
                } else {
                    // case 2: the current pose is NOT close to the closestPose, in which case we need to find the closest angle
                    double targetAngle = Math.atan2(closestPose.getY()- currentPose.getY(), closestPose.getX()- currentPose.getX());
                    double currentAngle = currentPose.getHeading();
                    // case 2a: tangential is closer, so we need to turn less
                    if (Math.abs(MathFunctions.angleWrap(targetAngle-currentAngle)) < Math.abs(MathFunctions.angleWrap(Math.PI- (targetAngle-currentAngle)))) {
                        kickToClosest = new SimplePathChain(
                                new SimplePath(currentPose, new Pose(closestPose.getX(), closestPose.getY(), targetAngle))
                        );
                    } else {
                        kickToClosest = new SimplePathChain(
                                new SimplePath(currentPose, new Pose(closestPose.getX(), closestPose.getY(), Math.PI-targetAngle))
                        );
                    }
                }
                drivetrain.kick(kickToClosest);
            }
        }

        gp1.update();

        if (gp1.rightBumperPressed()) {
            robot.shootCommand.start();
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
            SimplePathChain driveGate = new SimplePathChain(
                    new SimplePath(drivetrain.follower.getPose(), new Pose((alliance == Alliance.BLUE ? gatePose.getX()+15: gatePose.getX()-15), gatePose.getY(), gatePose.getHeading())),
                    new SimplePath(new Pose((alliance == Alliance.BLUE ? gatePose.getX()+15: gatePose.getX()-15), gatePose.getY(), gatePose.getHeading()), gatePose)
            );

            drivetrain.followPath(driveGate);
        }

        // y button: park
        if (gp1.yPressed()) {
            SimplePathChain park = new SimplePathChain(
                    new SimplePath(drivetrain.follower.getPose(), parkPose)
            );

            drivetrain.followPath(park);
        }

        // safety for autodrive
        if (gamepad1.leftStickButtonWasPressed() || gamepad1.rightStickButtonWasPressed()) {
            drivetrain.breakFollowing();
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
            robot.turret.setFeedforward(0);

            telemetry.addData("pitch", values[1]);
            telemetry.addData("velocity", values[2]);
            telemetry.addData("current velocity", robot.shooter.getCurrentVelocity());
        } else {
            double[] values = sotm.calculateAzimuthThetaVelocity(drivetrain.follower.getPose(), drivetrain.follower.getVelocity());
            robot.turret.setTarget(values[0]+turretOffset);
            robot.shooter.setShooterPitch(values[1]);
            robot.shooter.setTargetVelocity(values[2]);
            robot.turret.setFeedforward(0);

            telemetry.addData("pitch", values[1]);
            telemetry.addData("velocity", values[2]);
            telemetry.addData("current velocity", robot.shooter.getCurrentVelocity());
        }

        telemetry.addData("pose", drivetrain.follower.getPose());
        telemetry.addData("goal pose", drivetrain.getGoalPose());
        telemetry.addData("current state", drivetrain.getState());
        telemetry.addLine("ROBOT NOT IN SHOOTING POSE: " + !robot.inShootingZone(drivetrain.follower.getPose()));
        telemetry.addLine("INTAKE FULL: " + robot.intake.intakeFull());
        telemetry.addLine("NOT AutoDriving: " + !isAutoDriving);
        telemetry.addLine("ROBOT is idle: " + (robotState == RobotState.IDLE));

        if (alliance == Alliance.BLUE) {
            drivetrain.update(normalizeInput(gp1.getLeftStickY()*longitudinalSpeed),
                    normalizeInput(-gp1.getLeftStickX()*lateralSpeed),
                    normalizeInput(gp1.getRightStickX()*rotationSpeed));
        } else if (alliance == Alliance.RED) {
            drivetrain.update(normalizeInput(-gp1.getLeftStickY()*longitudinalSpeed),
                    normalizeInput(gp1.getLeftStickX()*lateralSpeed),
                    normalizeInput(gp1.getRightStickX()*rotationSpeed));
        }
        robot.update();
        telemetry.update();
    }


    public void start() {
        robot.initPositions();
        robot.start();
        drivetrain.start();
    }

    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, drivetrain.follower.getPose());
    }
}
