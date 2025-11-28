package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="weak motor tuner")
public class WeakMotorTuner extends OpMode {
    private DcMotorEx fl, bl, fr, br;

    @Override
    public void loop() {
        if (gamepad1.right_stick_x < 0) {
            fl.setPower(-gamepad1.right_stick_x);
            br.setPower(gamepad1.right_stick_x);
        }

        if (gamepad1.right_stick_x > 0) {
            fr.setPower(-gamepad1.right_stick_x);
            bl.setPower(gamepad1.right_stick_x);
        }

        telemetry.addLine("Push the right joystick to the left");
        telemetry.addLine("If the robot turns clockwise, the right back is weak. Otherwise, the front left is weak.");

        telemetry.addLine("Push the right joystick to the right");
        telemetry.addLine("If the robot turns clockwise, the right front is weak. Otherwise, the back left is weak.");

        telemetry.update();
    }

    @Override
    public void init() {
        fl = this.hardwareMap.get(DcMotorEx.class, "front_left_drive");
        bl = this.hardwareMap.get(DcMotorEx.class, "back_left_drive");
        fr = this.hardwareMap.get(DcMotorEx.class, "front_right_drive");
        br = this.hardwareMap.get(DcMotorEx.class, "back_right_drive");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {

    }
}
