package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.robot.robots.TeleopRobot;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

public class ExampleTeleop extends OpMode {
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
        // wait i have this brilliant idea
        // so we can have another statemachine here
        // we can use a button for cycling. Transition(() -> gp1.rightBumperPressed())
        // and a back button: transition(() -> gp1.xPressed()), "name of prev state"
        gp1.update();
        gp2.update();

        if (gp1.leftBumperPressed()) {
            robot.scoreSpecimen.start();
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
        follower.startTeleopDrive();
        robot.start();
    }
}
