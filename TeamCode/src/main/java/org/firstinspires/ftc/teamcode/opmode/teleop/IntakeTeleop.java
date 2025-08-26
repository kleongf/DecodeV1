package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.robots.TeleopRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.ActiveIntake;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
@TeleOp(name="Active Intake Teleop")
public class IntakeTeleop extends OpMode {
    private ActiveIntake activeIntake;
    private SmartGamepad gp1;


    @Override
    public void init() {
        activeIntake = new ActiveIntake(hardwareMap);
        gp1 = new SmartGamepad(gamepad1);
    }
    @Override
    public void loop() {
        gp1.update();

        if (gp1.leftBumperPressed()) {
            activeIntake.setRunning(true);
        }
        if (gp1.rightBumperPressed()) {
            activeIntake.setRunning(false);
        }

        if (gp1.x()) {
            activeIntake.setTargetColor("red");
        }

        if (gp1.y()) {
            activeIntake.setTargetColor("blue");
        }

        if (gp1.a()) {
            activeIntake.setTargetColor("yellow");
        }

        activeIntake.update();
    }

    @Override
    public void start() {
        activeIntake.start();
    }
}
