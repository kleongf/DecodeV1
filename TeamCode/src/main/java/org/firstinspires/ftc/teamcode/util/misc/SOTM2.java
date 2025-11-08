package org.firstinspires.ftc.teamcode.util.misc;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;

public class SOTM2 {
    private Pose goal;
    private LUT thetaLUT;
    private LUT velocityLUT;
    public boolean isBlue = true;
    private double radius = 0.036; // 36 mm radius, 72mm wheel
    private double speedCoefficient = 0.7; // accounts for compression and stuff idk
    public SOTM2(Pose goal) {
        this.goal = goal;
        // TODO: add tuned values here for theta and velocity
        // also these go from 0 to 20, where 20 is our max for some reason (25-45)
        thetaLUT = new LUT();
        thetaLUT.addData(38, Math.toRadians(0));
        thetaLUT.addData(48, Math.toRadians(4));
        thetaLUT.addData(58, Math.toRadians(8));
        thetaLUT.addData(68, Math.toRadians(12));
        thetaLUT.addData(78, Math.toRadians(14));
        thetaLUT.addData(88, Math.toRadians(16));
        thetaLUT.addData(98, Math.toRadians(18));
        thetaLUT.addData(108, Math.toRadians(19));
        thetaLUT.addData(118, Math.toRadians(20));
        thetaLUT.addData(128, Math.toRadians(20));
        thetaLUT.addData(138, Math.toRadians(22));
        thetaLUT.addData(148, Math.toRadians(22));

        velocityLUT = new LUT();
        velocityLUT.addData(148, 2180);
        velocityLUT.addData(138, 2080);
        velocityLUT.addData(128, 2040);
        velocityLUT.addData(118, 1960);
        velocityLUT.addData(108, 1870);
        velocityLUT.addData(98, 1800);
        velocityLUT.addData(88, 1700);
        velocityLUT.addData(78, 1600);
        velocityLUT.addData(68, 1540);
        velocityLUT.addData(58, 1490);
        velocityLUT.addData(48, 1400);
        velocityLUT.addData(38, 1200);

    }
    private double calculateLinearVelocityInches(double ticksPerSecond) {
        return (ticksPerSecond * 2 * Math.PI / 28.0) * radius * speedCoefficient * (39.3701);
    }
    public double[] calculateAzimuthThetaVelocity(Pose robotPose, Vector robotVelocity) {
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);

        // add 20 degrees because thats the original dont ask why
//        double t = dist / (calculateLinearVelocityInches(velocityLUT.getValue(dist)) * Math.cos(thetaLUT.getValue(dist)+Math.toRadians(20)));
//        t += 0.5; // it takes about 0.5 seconds to shoot
//
//        double dxCorr = dx + robotVelocity.getXComponent() * t;
//        double dyCorr = dy - robotVelocity.getYComponent() * t;
//        double distCorr = Math.hypot(dxCorr, dyCorr);
//
//        // coord system conversion to turret angle pov
//
//        double azimuth = Math.atan2(-dxCorr, dyCorr) - robotPose.getHeading() + Math.toRadians(90);
//        double theta = thetaLUT.getValue(distCorr);
//        double velocity = velocityLUT.getValue(distCorr);
        // TODO: offset added for blue side - will have to be subtracted for red side
        // actually maybe it shouldn't be a constant, it should be inversely proportional to distance.
        // as distance increases, the offset decreases.
        // so it's like 6 degrees * (dist - minDist)/maxdist if dist > minDist else 6 degrees
        double minDist = 48;
        double offset = isBlue ? Math.toRadians(-1) : Math.toRadians(1);
                // + Math.toRadians(-1.1) * (dist/minDist);
        // dist > minDist ? Math.toRadians(-4) * (minDist/dist) : Math.toRadians(-4);

        Vector v = com.pedropathing.pathgen.MathFunctions.subtractVectors(goal.getVector(), robotPose.getVector());
        Vector u = robotVelocity;

        // (u ⋅ v / |v|²) * v
        Vector projuv = com.pedropathing.pathgen.MathFunctions.scalarMultiplyVector(v, com.pedropathing.pathgen.MathFunctions.dotProduct(u, v) / com.pedropathing.pathgen.MathFunctions.dotProduct(v, v));

        // get the tangential component
        Vector vTangential = MathFunctions.subtractVectors(u, projuv);

        // if the vectors are in the same direction, then we should subtract the radial velocity
        // vectors are in the same direction if their dot product is positive, so dot it with the goal vector.
        double velToGoal = MathFunctions.dotProduct(projuv, v) > 0 ? projuv.getMagnitude() : -projuv.getMagnitude();

        // now subtract it from the velocity
        // v = r * omega, omega = v (inches to meters) / r (meters) -> divide by 2pi and the multiply by 28. also account for angle
        double inchesToTicks = (velToGoal * (1/39.3701) / radius) / (2 * Math.PI) * 28 * (1/speedCoefficient) * (1/Math.cos(Math.toRadians(20)+thetaLUT.getValue(dist)));

        double velocity = velocityLUT.getValue(dist) - inchesToTicks;
        // 0.2 seconds before shooting: always

        double timestep = 0.2 + 4 * (dist / (calculateLinearVelocityInches(velocityLUT.getValue(dist)) * Math.cos(thetaLUT.getValue(dist)+Math.toRadians(20))));
        System.out.println("Timestep: " + timestep);
        // double timestep = 0.5; // seconds

        double azimuth = Math.atan2(-(dx+vTangential.getXComponent()*timestep), (dy+vTangential.getYComponent()*timestep)) - robotPose.getHeading() + Math.toRadians(90) + offset;
        double theta = thetaLUT.getValue(dist);
        // double velocity = velocityLUT.getValue(dist);

        return new double[] {azimuth, theta, velocity};
    }
}
