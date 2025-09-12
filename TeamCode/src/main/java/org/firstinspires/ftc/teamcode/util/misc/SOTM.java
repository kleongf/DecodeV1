package org.firstinspires.ftc.teamcode.util.misc;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;

import org.firstinspires.ftc.teamcode.util.purepursuit.Pose2D;

public class SOTM {
    private final double g = 9.81; // gravity. maybe i should convert everything to inches/s^2.
    private final double zi = 0.3; // meters the shooter is above ground. again prob inches
    private final double zf = 1.5; // target height maybe should be inches
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

        // TODO: could result in a div0 error
        double t = (dz) / (vMin * Math.cos(thetaMin));

        double driftX = robotVelocity.getXComponent() * t;
        double driftY = robotVelocity.getYComponent() * t;

        double dEffX = dx - driftX;
        double dEffY = dy - driftY;
        double distEff = Math.hypot(dx, dy);
        double rho = Math.atan2(dEffY, dEffX);

        double[] vThetaEff = computeMinSpeedAngle(distEff, dz);
        double vMinEff = vThetaEff[0];
        double thetaMinEff = vThetaEff[1];

        return new double[]{rho, thetaMinEff, vMinEff};
    }

    public double[] computeMinSpeedAngle(double dist, double dz) {
        double r = Math.hypot(dist, dz);
        double v = Math.sqrt(g * (r + dz));
        double theta = Math.atan2((dz + r), dist);

        return new double[] {v, theta};
    }
}
