package org.firstinspires.ftc.teamcode.util.misc;
public class ProjectileODE implements Derivative {

    private final double g;
    private final double m;
    private final double c;

    public ProjectileODE(double g, double m, double c) {
        this.g = g;
        this.m = m;
        this.c = c;
    }

    @Override
    public void eval(double t, double[] y, double[] dydt) {

        double vx = y[3];
        double vy = y[4];
        double vz = y[5];

        double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);

        // Position derivatives
        dydt[0] = vx;
        dydt[1] = vy;
        dydt[2] = vz;

        // Velocity derivatives
        dydt[3] = -(c / m) * speed * vx;
        dydt[4] = -(c / m) * speed * vy;
        dydt[5] = -(c / m) * speed * vz - g;
    }
}

