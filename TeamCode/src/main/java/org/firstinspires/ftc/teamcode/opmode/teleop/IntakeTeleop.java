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
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
@TeleOp(name="Intake Teleop")
public class IntakeTeleop extends OpMode {
    private Intake intake;
    private SmartGamepad gp1;


    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        gp1 = new SmartGamepad(gamepad1);
    }
    @Override
    public void loop() {
        gp1.update();

        if (gp1.leftBumperPressed()) {
            intake.intakeOn = true;
        }
        if (gp1.rightBumperPressed()) {
            intake.intakeOn = false;
        }

        if (gp1.dpadUpPressed()) {
            intake.releaseCenter();
        }

        if (gp1.dpadLeftPressed()) {
            intake.releaseLeft();
        }

        if (gp1.dpadRightPressed()) {
            intake.releaseRight();
        }

        if (gp1.dpadDownPressed()) {
            intake.reset();
        }

        intake.update();
    }

    @Override
    public void start() {
        intake.start();
    }
}
