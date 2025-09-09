package org.firstinspires.ftc.teamcode.util.controllers;

public class TBHController {
    private double target;
    private double motorPower = 0;
    private double kG;
    private double maxPower = 1;
    private double lastError = 0;
    private double tbh = 0;

    public TBHController(double kg, double mp) {
        kG = kg;
        maxPower = mp;
    }

    public double calculate(double currentValue) {

        double error = target - currentValue;

        motorPower += kG * error;

        motorPower = clamp(motorPower);

        if (Math.signum(lastError) != Math.signum(error)) {
            motorPower = 0.5 * (motorPower + tbh);
            tbh = motorPower;

            lastError = error;
        }

        return motorPower;

    }
    public void setTarget(double x) {
        lastError = target - x;
        target = x;
    }
    private double clamp(double input) {
        if (input > maxPower) {
            return maxPower;
        }
        if (input < -maxPower) {
            return -maxPower;
        }
        return input;
    }

    public void setGain(double x) {
        kG = x;
    }

    public double getError(double current) {
        return target - current;
    }

    public boolean atTarget(double current, double tolerance) {
        return Math.abs(target - current) <= tolerance;
    }
}


