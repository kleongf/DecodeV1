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
        thetaLUT.addData(48, Math.toRadians(0));
        thetaLUT.addData(58, Math.toRadians(4));
        thetaLUT.addData(68, Math.toRadians(6));
        thetaLUT.addData(78, Math.toRadians(10));
        thetaLUT.addData(88, Math.toRadians(12));
        thetaLUT.addData(98, Math.toRadians(15));
        thetaLUT.addData(108, Math.toRadians(17));
        thetaLUT.addData(118, Math.toRadians(20));
        thetaLUT.addData(128, Math.toRadians(20));
        thetaLUT.addData(138, Math.toRadians(20));
        thetaLUT.addData(148, Math.toRadians(20));

        velocityLUT = new LUT();
        velocityLUT.addData(148, 2150);
        velocityLUT.addData(138, 2090);
        velocityLUT.addData(128, 1970);
        velocityLUT.addData(118, 1850);
        velocityLUT.addData(108, 1780);
        velocityLUT.addData(98, 1700);
        velocityLUT.addData(88, 1600);
        velocityLUT.addData(78, 1500);
        velocityLUT.addData(68, 1500);
        velocityLUT.addData(58, 1350);
        velocityLUT.addData(48, 1300);

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
