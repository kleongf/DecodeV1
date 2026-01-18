package org.firstinspires.ftc.teamcode.util.misc;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;

import java.util.ArrayList;
import java.util.List;

public class SOTM2 {
    private Pose goal;
    private LUT thetaLUT;
    private LUT velocityLUT;
    private double radius = 0.036; // 36 mm radius, 72mm wheel
    public double timeScaleFactor = 1;
    public double constantTimeFactor = 0.0;
    public double offsetFactor = -0.02;

    public SOTM2(Pose goal) {
        this.goal = goal;

        thetaLUT = new LUT();
        // thetaLUT.addData(163, Math.toRadians(17));
        thetaLUT.addData(155, Math.toRadians(15));
        // thetaLUT.addData(153, Math.toRadians(17));
        thetaLUT.addData(148, Math.toRadians(15));
        // thetaLUT.addData(143, Math.toRadians(16));
        thetaLUT.addData(138, Math.toRadians(15));
        // thetaLUT.addData(133, Math.toRadians(16));
        thetaLUT.addData(128, Math.toRadians(15));
        thetaLUT.addData(118, Math.toRadians(14));
        thetaLUT.addData(108, Math.toRadians(13));
        thetaLUT.addData(98, Math.toRadians(11));
        thetaLUT.addData(88, Math.toRadians(9));
        thetaLUT.addData(78, Math.toRadians(6));
        thetaLUT.addData(68, Math.toRadians(3));
        thetaLUT.addData(58, Math.toRadians(0));
        thetaLUT.addData(53, Math.toRadians(0));

        velocityLUT = new LUT();
        // velocityLUT.addData(163, 1520+70);
        velocityLUT.addData(155, 1580);
        // velocityLUT.addData(153, 1680);
        velocityLUT.addData(148, 1540);
        // velocityLUT.addData(143, 1560);
        velocityLUT.addData(138, 1460);
        // velocityLUT.addData(133, 1420+70);
        velocityLUT.addData(128, 1420);
        velocityLUT.addData(118, 1360);
        velocityLUT.addData(108, 1300);

        velocityLUT.addData(98, 1260);
        velocityLUT.addData(88, 1180);
        velocityLUT.addData(78, 1140);
        velocityLUT.addData(68, 1100);
        velocityLUT.addData(58, 1080);
        velocityLUT.addData(53, 1140);

    }

    private double calculateLinearVelocityMeters(double ticksPerSecond) {
        return (ticksPerSecond * 2 * Math.PI / 28.0) * radius;
    }

    private double[] calculateAzimuthThetaVelocity(Pose robotPose, Pose goalPose) {
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);

        boolean isBlue = goal.getX() == 0;
        double offset = isBlue ? offsetFactor : -offsetFactor;

        double azimuth = Math.atan2(-dx, dy) - robotPose.getHeading() + Math.toRadians(90) + offset;
        double theta = thetaLUT.getValue(dist);
        double velocity = velocityLUT.getValue(dist);

        return new double[] {azimuth, theta, velocity};
    }

    public double[] calculateAzimuthThetaVelocity(Pose robotPose, Vector robotVelocity) {
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);

        double timestep = timeScaleFactor * simulateProjectileTOF(dist, thetaLUT.getValue(dist), velocityLUT.getValue(dist));
        Pose adjustedPose = new Pose(goal.getX() - robotVelocity.getXComponent() * timestep, goal.getY() - robotVelocity.getYComponent() * timestep);

        return calculateAzimuthThetaVelocity(robotPose, adjustedPose);
    }

    private double simulateProjectileTOF(double dist, double theta, double velocityTicks) {
        double m = 0.07845; // mass of ball in kg
        double v = calculateLinearVelocityMeters(velocityTicks); // ticks -> linear velocity
        double c = 0.5 * 1 * 1.225 * 0.01216604657; // 1/2 Cd * rho * cross sectional area in m^2

        theta += Math.toRadians(34); // because of weird offset trust
        double vx = v * Math.cos(theta); // x velocity
        double vy = v * Math.sin(theta); // y velocity

        // simulation constants!
        double dt = 0.001;
        double g = 9.8;
        double d = dist / 39.3701; // converting to meters
        double MAX_ITERATIONS = 10000;

        double x = 0;

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            double ax = (-c * Math.hypot(vx, vy) * vx) / m;
            double ay = (m * -g -c * Math.hypot(vx, vy) * vy) / m;
            vx = vx + ax * dt;
            vy = vy + ay * dt;

            x += vx * dt;

            if (x >= d) {
                return i/1000d;
            }
        }

        return 0.5; // just in case
    }
}

