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
        thetaLUT.addData(12, Math.toRadians(0));
        thetaLUT.addData(22, Math.toRadians(0));
        thetaLUT.addData(32, Math.toRadians(0));
        thetaLUT.addData(42, Math.toRadians(7));
        thetaLUT.addData(52, Math.toRadians(10));
        thetaLUT.addData(62, Math.toRadians(12));
        thetaLUT.addData(72, Math.toRadians(15));
        thetaLUT.addData(82, Math.toRadians(20));
        thetaLUT.addData(92, Math.toRadians(20));
        thetaLUT.addData(102, Math.toRadians(20));
        thetaLUT.addData(112, Math.toRadians(20));
        thetaLUT.addData(122, Math.toRadians(20));
        thetaLUT.addData(133, Math.toRadians(20));

        velocityLUT = new LUT();
        velocityLUT.addData(133, 2400);
        velocityLUT.addData(122, 2250);
        velocityLUT.addData(112, 2125);
        velocityLUT.addData(102, 2000);
        velocityLUT.addData(92, 1900);
        velocityLUT.addData(82, 1800);
        velocityLUT.addData(72, 1700);
        velocityLUT.addData(62, 1600);
        velocityLUT.addData(52, 1500);
        velocityLUT.addData(42, 1420);
        velocityLUT.addData(32, 1400);
        velocityLUT.addData(22, 1500);
        velocityLUT.addData(12, 1500);
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
