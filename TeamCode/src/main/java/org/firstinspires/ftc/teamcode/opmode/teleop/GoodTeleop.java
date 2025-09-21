package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.robots.TeleopRobot;
import org.firstinspires.ftc.teamcode.robot.robots.TeleopRobotV1;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.misc.SOTM2;
import org.firstinspires.ftc.teamcode.util.misc.Target;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.HashMap;
import java.util.Objects;

@TeleOp(name="first good teleop GOOD?")
public class GoodTeleop extends OpMode {
    private int state = 1;
    private VoltageCompFollower follower;
    private double longitudinalSpeed = 1.0, lateralSpeed = 1.0, rotationSpeed = 1.0;
    private TeleopRobotV1 robot;
    private final Pose startPose = new Pose(32, 7, Math.toRadians(90));
    private final Pose goalPose = new Pose(12, 132, Math.toRadians(45));
    private SmartGamepad gp1;
    private SmartGamepad gp2;
    private SOTM2 sotm2;
    private HashMap<Integer, StateMachine> stateMap;

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        robot = new TeleopRobotV1(hardwareMap);

        gp1 = new SmartGamepad(gamepad1);
        gp2 = new SmartGamepad(gamepad2);

        sotm2 = new SOTM2(goalPose);

        stateMap = new HashMap<>();
        stateMap.put(1, robot.prepareIntake);
        stateMap.put(2, robot.prepareShooting);
        stateMap.put(3, robot.startShooting);
    }
    @Override
    public void loop() {
        gp1.update();
        gp2.update();

        if (gp1.rightBumperPressed() && !robot.isBusy()) {
            Objects.requireNonNull(stateMap.get(state % 3)).start();
            state++;
        }

        // very cool back button should work
        if (gp1.leftBumperPressed()) {
            Objects.requireNonNull(stateMap.get(state-1 % 3)).start();
            state--;
        }

        double[] values = sotm2.calculateAzimuthThetaVelocity(follower.getPose(), follower.getVelocity());

        robot.turret.setTarget(values[0]);
        robot.shooter.setShooterPitch(values[1]);
        robot.shooter.setTargetVelocity(values[2]);

        follower.setTeleOpMovementVectors(
                    gp1.getLeftStickY() * longitudinalSpeed,
                    gp1.getLeftStickX() * lateralSpeed,
                    gp1.getRightStickX() * rotationSpeed
        );

        follower.update();
        robot.update();
    }

    @Override
    public void start() {
        // TODO: In the future, initPositions() should go here so we don't move on init
        robot.initPositions();
        follower.startTeleopDrive();
        robot.start();
    }
}
