package org.firstinspires.ftc.teamcode.util.purepursuit;

public class PIDFCoefficients {
    public double kp;
    public double ki;
    public double kd;
    public double kf;
    public PIDFCoefficients(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }
}
