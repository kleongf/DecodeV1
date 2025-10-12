package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.util.purepursuit.MathFunctions;

@Config
@TeleOp(name="Turret PIDF Tuner")
public class TurretTuner extends OpMode {
    private PIDFController controller;
    public static double kP = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double target = 0;
    public static double errorThreshold = 10;
    private double ticksPerRevolution = 1931; // 383.6*5 idk
    private double ticksPerRadian = ticksPerRevolution / (2 * Math.PI);

    public DcMotorEx turretMotor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDFController(kP, 0, kD, 0);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private double weirdAngleWrap(double radians) {
        while (radians > 0) {
            radians -= 2 * Math.PI;
        }
        while (radians < -2 * Math.PI) {
            radians += 2 * Math.PI;
        }
        // keep in mind that the result is in radians
        return radians;
    }

    @Override
    public void loop() {
        controller.setPIDF(kP, 0, kD, 0);
        double c = turretMotor.getCurrentPosition();
        // TODO: maybe anglewrap the target?
        double t = weirdAngleWrap(target) * ticksPerRadian;
        double power = controller.calculate(c, t);

//        double currentPos = turretMotor.getCurrentPosition() / ticksPerRadian;
//        //double targetPos = Math.toRadians(target);
//        double power = controller.calculate(currentPos, Math.toRadians(target));
        if (Math.abs(c-t) > errorThreshold) {
            double error = t-c;
            power += kF * Math.signum(error);
        }
        turretMotor.setPower(power);

        telemetry.addData("power", power);
        telemetry.addData("ticks", turretMotor.getCurrentPosition());
        telemetry.addData("current position", c);
        telemetry.addData("target", target);
        // telemetry.addData("ticks back to target", Math.toRadians(target) * ticksPerRadian);
        telemetry.update();
    }

}