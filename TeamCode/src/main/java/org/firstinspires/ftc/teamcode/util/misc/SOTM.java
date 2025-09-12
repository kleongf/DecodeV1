package org.firstinspires.ftc.teamcode.util.misc;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;

import org.firstinspires.ftc.teamcode.util.purepursuit.Pose2D;

public class SOTM {
    private final double g = 386.2205; // gravity, in/s^2
    private final double zi = 12; // in. our shooter height above ground
    private final double zf = 48; // in. the goal height
    private final double flywheelRadius = 1.5; // flywheel radius, in
    private final Pose goal;
    public SOTM(Pose goal) {
        this.goal = goal;
    }

    public double[] calculateRhoThetaVelocity(Pose robotPose, Vector robotVelocity) {
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);
        double dz = zf - zi;
        // double rho0 = Math.atan2(dy, dx);

        double[] vTheta = computeMinSpeedAngle(dist, dz);
        double vMin = vTheta[0];
        double thetaMin = vTheta[1];

        if ((vMin * Math.cos(thetaMin)) == 0) {
            // if we return 0, 0, 0, then just ignore it because its an error.
            return new double[] {0, 0, 0};
        }
        double t = (dz) / (vMin * Math.cos(thetaMin));

        double driftX = robotVelocity.getXComponent() * t;
        double driftY = robotVelocity.getYComponent() * t;

        double dEffX = dx - driftX;
        double dEffY = dy - driftY;
        double distEff = Math.hypot(dx, dy);
        double rho = Math.atan2(dEffY, dEffX);

        double[] vThetaCorr = computeMinSpeedAngle(distEff, dz);
        // returns rho (turret angle), theta (pitch angle), v (flywheel angular velocity, in/s).
        // v = r * omega, so omega = v/r.
        // for rho, we can just add this to the robot heading then normalize it with MathFunctions.
        return new double[] {rho, vThetaCorr[1], (vThetaCorr[0] / flywheelRadius)};
    }

    public double[] computeMinSpeedAngle(double dist, double dz) {
        double r = Math.hypot(dist, dz);
        double v = Math.sqrt(g * (r + dz));
        double theta = Math.atan2((dz + r), dist);

        return new double[] {v, theta};
    }
}
