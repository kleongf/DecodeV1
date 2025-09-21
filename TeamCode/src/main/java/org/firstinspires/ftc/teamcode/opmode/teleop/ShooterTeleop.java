package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.robots.TeleopRobot;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
@Disabled
@TeleOp(name="Shooter Teleop")
public class ShooterTeleop extends OpMode {
    private DcMotorEx vSlideMotorLeft;
    private DcMotorEx vSlideMotorRight;
    private DcMotorEx hSlideMotor;

    private SmartGamepad gp1;
    public boolean running = false;


    @Override
    public void init() {
        vSlideMotorLeft = hardwareMap.get(DcMotorEx.class, "VSlideLeft");
        vSlideMotorRight = hardwareMap.get(DcMotorEx.class, "VSlideRight");
        hSlideMotor = hardwareMap.get(DcMotorEx.class, "HSlide");
        vSlideMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        vSlideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        vSlideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vSlideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vSlideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        vSlideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gp1 = new SmartGamepad(gamepad1);
    }
    @Override
    public void loop() {
        gp1.update();

        if (gp1.xPressed()) {
            running = !running;
        }

        if (running) {
            vSlideMotorLeft.setPower(1);
            vSlideMotorRight.setPower(1);
            hSlideMotor.setPower(1);
        } else {
            vSlideMotorLeft.setPower(0);
            vSlideMotorRight.setPower(0);
            hSlideMotor.setPower(0);
        }

        telemetry.addData("Running", running);
        telemetry.update();
    }

    @Override
    public void start() {

    }
}
