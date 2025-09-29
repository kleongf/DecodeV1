package org.firstinspires.ftc.teamcode.opmode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;

@Autonomous(name="apriltag detection")
public class Apriltag extends OpMode {
    private AutonomousRobot robot;

    @Override
    public void init() {
        robot = new AutonomousRobot(hardwareMap);
    }

    @Override
    public void loop() {
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        robot.start();
    }
}

