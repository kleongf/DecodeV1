package org.firstinspires.ftc.teamcode.util.misc;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;

import java.util.ArrayList;
import java.util.List;

public class SOTM {
    private Pose goal;
    private LUT thetaLUT;
    private LUT velocityLUT;
    private double radius = 0.036; // 36 mm radius, 72mm wheel
    private double radiusBall = 0.06223; // 2.45 in

    public SOTM(Pose goal) {
        this.goal = goal;

        thetaLUT = new LUT();
        thetaLUT.addData(53, Math.toRadians(0));
        thetaLUT.addData(58, Math.toRadians(7));
        thetaLUT.addData(68, Math.toRadians(13));
        thetaLUT.addData(78, Math.toRadians(14));
        thetaLUT.addData(88, Math.toRadians(15));
        thetaLUT.addData(98, Math.toRadians(16));
        thetaLUT.addData(108, Math.toRadians(17));
        thetaLUT.addData(118, Math.toRadians(18));
        thetaLUT.addData(128, Math.toRadians(18));
        thetaLUT.addData(138, Math.toRadians(19));
        thetaLUT.addData(148, Math.toRadians(19));
        thetaLUT.addData(158, Math.toRadians(19));

        velocityLUT = new LUT();
        velocityLUT.addData(158, 1560);
        velocityLUT.addData(148, 1520);
        velocityLUT.addData(138, 1500);
        velocityLUT.addData(128, 1460);
        velocityLUT.addData(118, 1400);
        velocityLUT.addData(108, 1380);
        velocityLUT.addData(98, 1360);
        velocityLUT.addData(88, 1280);
        velocityLUT.addData(78, 1240);
        velocityLUT.addData(68, 1220);
        velocityLUT.addData(58, 1180);
        velocityLUT.addData(53, 1160);

    }
    private double calculateLinearVelocityInches(double ticksPerSecond) {
        return (ticksPerSecond * 2 * Math.PI / 28.0) * radius * (39.3701);
    }
    public double[] calculateAzimuthThetaVelocity(Pose robotPose, Vector robotVelocity) {
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);

        boolean isBlue = goal.getX() == 0;
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
        double inchesToTicks = (velToGoal * (1/39.3701) / radius) / (2 * Math.PI) * 28 * (1/Math.cos(Math.toRadians(28)+thetaLUT.getValue(dist))); // (1/Math.cos(Math.toRadians(20)+thetaLUT.getValue(dist)))

        double velocity = velocityLUT.getValue(dist) - inchesToTicks;
        // 0.2 seconds before shooting: always

        double timestep = 2 * (dist / (calculateLinearVelocityInches(velocityLUT.getValue(dist)) * Math.cos(thetaLUT.getValue(dist)+Math.toRadians(28))));

        // blue perspective:
        // pure angle to goal. from small angles, it overshoots to the left (from blue perspective this is positive turret),
        double angleToGoal = Math.atan2(-(dx-vTangential.getXComponent()*timestep), (dy-vTangential.getYComponent()*timestep));
        double offset = isBlue ? (angleToGoal - Math.PI / 4) * 0.15 : (angleToGoal + Math.PI / 4) * 0.15;
        // when angle is big, aim more left (which is positive direction), when it is small, aim more right (negative direction)
        // opposite for red, and all this helps i guess? backboard area is better when we higher so it makes sense idk
        // what do we count as 0? i think we count it as the 45 degree position, which i suppose is
        // for red it is from the negative 45 i think, and let's offset everything my like 6-7% as a test

        double azimuth = Math.atan2(-(dx-vTangential.getXComponent()*timestep), (dy-vTangential.getYComponent()*timestep)) - robotPose.getHeading() + Math.toRadians(90) + offset;
        double theta = thetaLUT.getValue(dist);

        return new double[] {azimuth, theta, velocity};
    }

    private List<Integer> findALlOccurences(List<String> motif, String target) {
        List<Integer> occurrences = new ArrayList<>();
        for (int i = 0; i < motif.size(); i++) {
            if (motif.get(i).equals(target)) {
                occurrences.add(i);
            }
        }
        return occurrences;
    }

    // returns meters/s
    private double ticksToLinearVelocity(double x) {
        // 1. convert to radians/s
        // 2. multiply by r
        // 3. multiply by R/r because it is the ball's velocity, not the wheel
        return (x * (2 * Math.PI) / 28) * (radius) * (radius / radiusBall);
    }
    private double calculateAngle(double v, double x, double y) {
        // only concern is square root discriminant. if negative, no solution
        double numerator = Math.pow(v, 2) - Math.sqrt(Math.pow(v, 4) - (Math.pow(9.81, 2) * Math.pow(x, 2)) - 2 * y * Math.pow(v, 2));
        double denominator = 9.81 * x;
        return Math.atan2(numerator, denominator);
    }

    private double calculateFinalAngle(double dist, double scaleFactor) {
        double currentTicks = velocityLUT.getValue(dist);
        double newTicks = currentTicks + scaleFactor * 100;
        double currentAngle = thetaLUT.getValue(dist) + Math.toRadians(28);

        double v0 = ticksToLinearVelocity(currentTicks);
        double x0 = dist * (1/39.3701);

        double t = x0 / (v0 * Math.cos(currentAngle));
        // height at distance = x
        double y0 = v0 * Math.sin(currentAngle) * t - 0.5 * 9.81 * t * t;

        // now plug in the y for the adjusted velocity, adjusted for the LUT
        double v1 = ticksToLinearVelocity(newTicks);
        return calculateAngle(v1, x0, y0) - Math.toRadians(28);
    }

    private double calculateFinalVelocityTicks(double dist, double scaleFactor) {
        return velocityLUT.getValue(dist) + scaleFactor * 100;
    }

    // calculates a scale factor for velocity. The more negative a number is, the higher angle and lower velocity it should be
    // if its high then we use a high velocity and low angle. we can change the factor.
    public List<List<Double>> calculateAzimuthThetaVelocityAirSort(Pose robotPose, Vector robotVelocity, List<String> artifactOrder, List<String> motif) {
        List<Integer> greenActualOrder = findALlOccurences(artifactOrder, "G");
        List<Integer> greenMotifOrder = findALlOccurences(motif, "G");

        List<Integer> purpleActualOrder = findALlOccurences(artifactOrder, "P");
        List<Integer> purpleMotifOrder = findALlOccurences(motif, "P");

        int scaleFactor0;
        int scaleFactor1;
        int scaleFactor2;

        if (artifactOrder.get(0).equals("G")) {
            scaleFactor0 = greenMotifOrder.get(0) - greenActualOrder.get(0);
            scaleFactor1 = purpleMotifOrder.get(0) - purpleActualOrder.get(0);
            scaleFactor2 = purpleMotifOrder.get(1) - purpleActualOrder.get(1);
        } else if (artifactOrder.get(1).equals("G")) {
            scaleFactor0 = purpleMotifOrder.get(0) - purpleActualOrder.get(0);
            scaleFactor1 = greenMotifOrder.get(0) - greenActualOrder.get(0);
            scaleFactor2 = purpleMotifOrder.get(1) - purpleActualOrder.get(1);
        } else {
            scaleFactor0 = purpleMotifOrder.get(0) - purpleActualOrder.get(0);
            scaleFactor1 = purpleMotifOrder.get(1) - purpleActualOrder.get(1);
            scaleFactor2 = greenMotifOrder.get(0) - greenActualOrder.get(0);
        }
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);
        double azimuth = Math.atan2(-dx, dy) - robotPose.getHeading() + Math.toRadians(90);

        // return a 3 x 3 array
        // 1st dimension: each ball
        // second dimension: azimuth theta velocity
        List<List<Double>> out = new ArrayList<>();

        List<Double> ball0 = new ArrayList<>();
        ball0.add(azimuth);
        ball0.add(calculateFinalAngle(dist, scaleFactor0));
        ball0.add(calculateFinalVelocityTicks(dist, scaleFactor0));

        List<Double> ball1 = new ArrayList<>();
        ball1.add(azimuth);
        ball1.add(calculateFinalAngle(dist, scaleFactor1));
        ball1.add(calculateFinalVelocityTicks(dist, scaleFactor1));

        List<Double> ball2 = new ArrayList<>();
        ball2.add(azimuth);
        ball2.add(calculateFinalAngle(dist, scaleFactor2));
        ball2.add(calculateFinalVelocityTicks(dist, scaleFactor2));

        out.add(ball0);
        out.add(ball1);
        out.add(ball2);

        // this method is the same but provides three values
        // TODO: methods in the robot classes that allow us to shootFirst, shootSecond, shootThird
        return out;
    }
}
