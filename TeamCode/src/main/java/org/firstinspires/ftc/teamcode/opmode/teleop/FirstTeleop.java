package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.robots.TeleopRobot;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
@TeleOp(name="first teleop")
public class FirstTeleop extends OpMode {
    private VoltageCompFollower follower;
    private double longitudinalSpeed = 1.0, lateralSpeed = 1.0, rotationSpeed = 1.0;
    private TeleopRobot robot;
    private final Pose startPose = new Pose(9, 66, Math.toRadians(0));
    private SmartGamepad gp1;
    private SmartGamepad gp2;


    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        robot = new TeleopRobot(hardwareMap);

        gp1 = new SmartGamepad(gamepad1);
        gp2 = new SmartGamepad(gamepad2);

        try {
            sleep(1000);
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

        if (gp1.leftBumperPressed()) {
            robot.prepareIntake.start();
        }

        if (gp1.rightBumperPressed()) {
            robot.prepareShooting.start();
        }

        if (gp1.xPressed()) {
            robot.shootGreen.start();
        }

        if (gp1.yPressed()) {
            robot.shootPurple.start();
        }

        follower.update();
        follower.setTeleOpMovementVectors(
                gp1.getLeftStickY() * longitudinalSpeed,
                gp1.getLeftStickX() * lateralSpeed,
                gp1.getRightStickX() * rotationSpeed
        );
        robot.update();
    }

    @Override
    public void start() {
        // TODO: In the future, initPositions() should go here so we don't move on init
        follower.startTeleopDrive();
        robot.start();
    }
}
