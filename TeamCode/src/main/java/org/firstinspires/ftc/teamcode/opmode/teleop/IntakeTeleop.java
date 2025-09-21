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
import org.firstinspires.ftc.teamcode.robot.subsystems.BetterIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
@TeleOp(name="Intake Teleop WORKING")
public class IntakeTeleop extends OpMode {
    private BetterIntake intake;
    private SmartGamepad gp1;


    @Override
    public void init() {
        intake = new BetterIntake(hardwareMap);
        gp1 = new SmartGamepad(gamepad1);
    }
    @Override
    public void loop() {
        gp1.update();

        if (gp1.leftBumperPressed()) {
            intake.setIntakeOn(!intake.isIntakeOn());
        }

        intake.update();
    }

    @Override
    public void start() {
        intake.start();
    }
}
