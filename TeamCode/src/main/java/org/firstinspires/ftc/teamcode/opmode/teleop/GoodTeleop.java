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
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.misc.Target;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
@TeleOp(name="first good teleop")
public class GoodTeleop extends OpMode {
    private int state = 1;
    private VoltageCompFollower follower;
    private double longitudinalSpeed = 1.0, lateralSpeed = 1.0, rotationSpeed = 1.0;
    private TeleopRobotV1 robot;
    private final Pose startPose = new Pose(32, 7, Math.toRadians(90));
    private SmartGamepad gp1;
    private SmartGamepad gp2;


    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        robot = new TeleopRobotV1(hardwareMap);

        gp1 = new SmartGamepad(gamepad1);
        gp2 = new SmartGamepad(gamepad2);

        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.initPositions();
    }
    @Override
    public void loop() {
        // TODO: have a robot is busy thing so that commands dont collide
        gp1.update();
        gp2.update();

        if (gp1.rightBumperPressed()) {
            if (state == 1) {
                robot.prepareIntake.start();
                state = 2;
            } else if (state ==2) {
                robot.prepareShooting.start();
                state = 3;
            } else if (state == 3) {
                robot.startShooting.start();
                state = 1;
            }
        }

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
        follower.startTeleopDrive();
        robot.start();
    }
}
