package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.controllers.TBHController;

@Config
@TeleOp(name="Flywheel TBH Tuner")
public class TBHTuner extends OpMode {
    private TBHController controller;
    public static double kG = 0;

    public static int target = 0;

    public DcMotorEx shooterMotor;
    public DcMotorEx shooterMotor2;

    @Override
    public void init() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            controller = new TBHController(0, 1);

            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            // shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // TODO: max velocity is 2800 ticks/s, best kG value 0.00001
    }

        @Override
        public void loop() {
            controller.setGain(kG);
            // TODO: keep this in mind
            // theoretically, since there are 2pi radians in a circle, max rpm = 2pi * (6000/60) = 200pi or about 628 rad/s.
            double currentVel = shooterMotor.getVelocity();
            controller.setTarget(target);
            double power = controller.calculate(currentVel);
            shooterMotor.setPower(power);
            shooterMotor2.setPower(power);

            telemetry.addData("power", power);

            telemetry.addData("current velocity", currentVel);
            telemetry.addData("target", target);
            telemetry.update();
        }

}
