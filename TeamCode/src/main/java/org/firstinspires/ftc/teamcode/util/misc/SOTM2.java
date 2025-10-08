package org.firstinspires.ftc.teamcode.util.misc;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;

import org.firstinspires.ftc.teamcode.util.purepursuit.MathFunctions;

public class SOTM2 {
    private Pose goal;
    private LUT thetaLUT;
    private LUT velocityLUT;
    private double radius = 0.036; // 36 mm radius, 72mm wheel
    private double speedCoefficient = 0.8; // accounts for compression and stuff idk
    public SOTM2(Pose goal) {
        this.goal = goal;
        // TODO: add tuned values here for theta and velocity
        // also these go from 0 to 20, where 20 is our max for some reason (25-45)
        thetaLUT = new LUT();
        thetaLUT.addData(24, Math.toRadians(0));
        thetaLUT.addData(34, Math.toRadians(8));
        thetaLUT.addData(44, Math.toRadians(11));
        thetaLUT.addData(54, Math.toRadians(13));
        thetaLUT.addData(64, Math.toRadians(13));
        thetaLUT.addData(74, Math.toRadians(13));
        thetaLUT.addData(84, Math.toRadians(14));
        thetaLUT.addData(94, Math.toRadians(14));
        thetaLUT.addData(104, Math.toRadians(15));
        thetaLUT.addData(114, Math.toRadians(16));
        thetaLUT.addData(124, Math.toRadians(16));
        thetaLUT.addData(134, Math.toRadians(16));

        velocityLUT = new LUT();
        velocityLUT.addData(134, 2250);
        velocityLUT.addData(124, 2150);
        velocityLUT.addData(114, 2050);
        velocityLUT.addData(104, 1950);
        velocityLUT.addData(94, 1850);
        velocityLUT.addData(84, 1750);
        velocityLUT.addData(74, 1680);
        velocityLUT.addData(64, 1640);
        velocityLUT.addData(54, 1600);
        velocityLUT.addData(44, 1550);
        velocityLUT.addData(34, 1450);
        velocityLUT.addData(24, 1430);
    }
    private double calculateLinearVelocity(double ticksPerSecond) {
        return (ticksPerSecond * 2 * Math.PI / 28.0) * radius * speedCoefficient;
    }
    public double[] calculateAzimuthThetaVelocity(Pose robotPose, Vector robotVelocity) {
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);

        System.out.println(dist);

//        double t = dist / (calculateLinearVelocity(velocityLUT.getValue(dist)) * Math.cos(thetaLUT.getValue(dist)));
//        double dxCorr = dx - robotVelocity.getXComponent() * t;
//        double dyCorr = dy - robotVelocity.getYComponent() * t;
//        double distCorr = Math.hypot(dxCorr, dyCorr);
//
//        // coord system conversion to turret angle pov
//
 //        double azimuth = Math.atan2(-dxCorr, dyCorr) - robotPose.getHeading() + Math.toRadians(90);
//        double theta = thetaLUT.getValue(distCorr);
//        double velocity = velocityLUT.getValue(distCorr);1111
        double azimuth = Math.atan2(-dx, dy) - robotPose.getHeading() + Math.toRadians(90);
        System.out.println(robotPose.getHeading());
        // System.out.println(Math.atan2(-dx, dy));
        double theta = thetaLUT.getValue(dist);
        double velocity = velocityLUT.getValue(dist);
        System.out.println(theta);
        System.out.println(velocity);



        return new double[] {azimuth, theta, velocity};
    }
}
