package org.firstinspires.ftc.teamcode.util.misc;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;

public class SOTM2 {
    private Pose goal;
    private LUT thetaLUT;
    private LUT velocityLUT;
    private double radius = 0.036; // 36 mm radius, 72mm wheel
    private double speedCoefficient = 0.8; // accounts for compression and stuff idk
    public SOTM2(Pose goal) {
        this.goal = goal;
        // TODO: add tuned values here for theta and velocity
        thetaLUT = new LUT();
        thetaLUT.addData(10, Math.toRadians(60));
        velocityLUT = new LUT();
    }
    private double calculateLinearVelocity(double ticksPerSecond) {
        return (ticksPerSecond * 2 * Math.PI / 28.0) * radius * speedCoefficient;
    }
    public double[] calculateAzimuthThetaVelocity(Pose robotPose, Vector robotVelocity) {
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);

        double t = dist / (calculateLinearVelocity(velocityLUT.getValue(dist)) * Math.cos(thetaLUT.getValue(dist)));
        double dxCorr = dx - robotVelocity.getXComponent() * t;
        double dyCorr = dy - robotVelocity.getYComponent() * t;
        double distCorr = Math.hypot(dxCorr, dyCorr);

        // coord system conversion to turret angle pov

        double azimuth = Math.atan2(-dxCorr, dyCorr) - robotPose.getHeading() + Math.toRadians(90);
        double theta = thetaLUT.getValue(distCorr);
        double velocity = velocityLUT.getValue(distCorr);

        return new double[] {azimuth, theta, velocity};
    }
}
