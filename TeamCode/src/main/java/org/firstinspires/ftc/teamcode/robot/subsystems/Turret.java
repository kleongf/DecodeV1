package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.controllers.HeadingPIDFController;
import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.util.purepursuit.MathFunctions;

public class Turret extends Subsystem {
    public DcMotorEx turretMotor;
    public PIDFController turretController;
    public double target = 0;
    private double ticksPerRevolution = 1931; // 383.6*5 idk
    private double ticksPerRadian = ticksPerRevolution / (2 * Math.PI);
    private double feedforward = 0;


    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretController = new PIDFController(0.005, 0, 0.00005, 0);
    }

    @Override
    public void update() {
        double c = turretMotor.getCurrentPosition();
        double t = weirdAngleWrap(target) * ticksPerRadian;

        double power = turretController.calculate(c, t);
        if (Math.abs(c-t) > 10) {
            double error = t-c;
            power += 0.01 * Math.signum(error);
        }
        power += feedforward;
        turretMotor.setPower(power);
    }

    public void setPDCoefficients(double p, double d) {
        turretController.setPIDF(p, 0, d, 0);
    }

    @Override
    public void start() {

    }

    public void setFeedforward(double x) {
        feedforward = x;
    }

    private double weirdAngleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        // keep in mind that the result is in radians
//        if (radians > Math.PI - Math.toRadians(20)) {
//            return Math.PI - Math.toRadians(20);
//        }
        return radians;
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTarget(double x) {
        target = x;
    }
    public boolean atTarget(double threshold) {
        return Math.abs(turretMotor.getCurrentPosition()-weirdAngleWrap(target) * ticksPerRadian) < threshold;
    }
}
