package org.firstinspires.ftc.teamcode.util.misc;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;

public class SOTM3 {
    private Pose goal;
    private LUT thetaLUT;
    private LUT velocityLUT;
    private double radius = 0.036; // 36 mm radius, 72mm diameter wheel
    // accounts for compression, rotation, air resistance, etc. not all rotation is converted into linear motion.
    private double speedCoefficient = 0.7;
    private double kV = 1.0;
    public SOTM3(Pose goal) {
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
    private double calculateLinearVelocityInches(double ticksPerSecond) {
        return (ticksPerSecond * 2 * Math.PI / 28.0) * radius * speedCoefficient * (39.3701);
    }

    public double[] calculateAzimuthFeedforwardThetaVelocity(Pose robotPose, Vector robotVelocity) {
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);

        double dt = 0.03; // looptime =about 30ms
        double dxNext = goal.getX() - (robotPose.getX() + robotVelocity.getXComponent() * dt);
        double dyNext = goal.getY() - (robotPose.getY() + robotVelocity.getYComponent() * dt);

        // we should never run into weird domain issues unless odo is off, but be safe
        double feedforward = kV * MathFunctions.normalizeAngle(Math.atan2(-dxNext, dyNext) - Math.atan2(-dx, dy));

        // get the velocity vector and project it onto the goal vector

        Vector v = MathFunctions.subtractVectors(goal.getVector(), robotPose.getVector());
        Vector u = robotVelocity;

        // (u ⋅ v / |v|²) * v
        Vector projuv = MathFunctions.scalarMultiplyVector(v, MathFunctions.dotProduct(u, v) / MathFunctions.dotProduct(v, v));

        // if the vectors are in the same direction, then we should subtract the radial velocity
        // vectors are in the same direction if their dot product is positive, so dot it with the goal vector.
        double velToGoal = MathFunctions.dotProduct(projuv, v) > 0 ? projuv.getMagnitude() : -projuv.getMagnitude();

        // now subtract it from the velocity
        // v = r * omega, omega = v (inches to meters) / r (meters) -> divide by 2pi and the multiply by 28
        double inchesToTicks = (velToGoal * (1/39.3701) / radius) / (2 * Math.PI) * 28 * (1/speedCoefficient);

        double velocity = velocityLUT.getValue(dist) - inchesToTicks;
        double azimuth = Math.atan2(-dx, dy) - robotPose.getHeading() + Math.toRadians(90);
        double theta = thetaLUT.getValue(dist);

        return new double[] {azimuth, feedforward, theta, velocity};
    }
}
