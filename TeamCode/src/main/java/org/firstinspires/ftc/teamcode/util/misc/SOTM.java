package org.firstinspires.ftc.teamcode.util.misc;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class SOTM {
    private Pose goal;
    private LUT thetaLUT;
    private LUT velocityLUT;
    private double radius = 0.036; // 36 mm radius, 72mm wheel
    private double radiusBall = 0.06223; // 2.45 in
    public double timeScaleFactor = 2.4;
    public double constantTimeFactor = 0.05;
    public double offsetFactor = 8; // think i found it! after doing some algebra
    private double MAX_ITERATIONS = 67;
    public double radialVelocityScaleFactor = 1.2; // made to match the timeScale b/c if we're using bad physics we may as well use it for both right?

    public SOTM(Pose goal) {
        this.goal = goal;

        thetaLUT = new LUT();
        thetaLUT.addData(163, Math.toRadians(15));
        thetaLUT.addData(158, Math.toRadians(15));
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
        velocityLUT.addData(163, 1500);
        velocityLUT.addData(158, 1480);
        // velocityLUT.addData(153, 1680);
        velocityLUT.addData(148, 1440);
        // velocityLUT.addData(143, 1560);
        velocityLUT.addData(138, 1400);
        // velocityLUT.addData(133, 1420+70);
        velocityLUT.addData(128, 1320);
        velocityLUT.addData(118, 1260);
        velocityLUT.addData(108, 1200);

        velocityLUT.addData(98, 1140);
        velocityLUT.addData(88, 1110);
        velocityLUT.addData(78, 1080);
        velocityLUT.addData(68, 1020);
        velocityLUT.addData(58, 960);
        velocityLUT.addData(53, 960);

    }
    private double calculateLinearVelocityInches(double ticksPerSecond) {
        return (ticksPerSecond * 2 * Math.PI / 28.0) * radius * (39.3701);
    }

    private double calculateTicksPerSecond(double linearVelocityInches) {
        return linearVelocityInches / ((2 * Math.PI / 28.0) * radius * 39.3701);
    }


    private double calculateLinearVelocityMeters(double ticksPerSecond) {
        return (ticksPerSecond * 2 * Math.PI / 28.0) * radius;
    }
    // how to make a SOTM tha accounts for the velocity with the hood instead of the speed, since speed can't update that quickly?
    // find the angle that increases the x velocity by the desired amount?
    // ex: velocity of wheel x = cos(angle) * wheel vel
    // x' = cos(theta') * wheel vel = x + extra v
    // so we have: x cos (thata') = x cos (theta) + extra v
    // theta' = cos^-1 (cos(theta)+extra v / x)

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
        double theta = thetaLUT.getValue(dist);
        // double newTheta = Math.acos(Math.cos(theta) + (radialVelocityScaleFactor * velToGoal/dist)) - Math.toRadians(34); // all in inches so its fine

        double inchesToTicks = radialVelocityScaleFactor * (velToGoal * (1/39.3701) / radius) / (2 * Math.PI) * 28 * (1/Math.cos(Math.toRadians(28)+thetaLUT.getValue(dist))); // (1/Math.cos(Math.toRadians(20)+thetaLUT.getValue(dist)))

        double velocity = velocityLUT.getValue(dist) - inchesToTicks;
        // double velocity = velocityLUT.getValue(dist);
        // 0.2s before shooting: always

        double timestep = constantTimeFactor + timeScaleFactor * (dist / (calculateLinearVelocityInches(velocityLUT.getValue(dist)) * Math.cos(thetaLUT.getValue(dist)+Math.toRadians(34))));
        // constantTimeFactor + timeScaleFactor * (dist / (calculateLinearVelocityInches(velocityLUT.getValue(dist)) * Math.cos(thetaLUT.getValue(dist)+Math.toRadians(28))));
                // simulateProjectileTOF(dist, thetaLUT.getValue(dist), velocityLUT.getValue(dist));

        // blue perspective:
        // pure angle to goal. from small angles, it overshoots to the left (from blue perspective this is positive turret),
        double angleToGoal = Math.atan2(-(dx-vTangential.getXComponent()*timestep), (dy-vTangential.getYComponent()*timestep));

        System.out.println("timestep: " + timestep);
        System.out.println("Tangential X: " + vTangential.getXComponent());
        System.out.println("Tangential Y: " + vTangential.getYComponent());
        // TODO: idea: offset is a bigger problem at far distances (more emphasized), what if we divide offset by distance at the end?
        // this would make more sense... given that it is based on arc length
        // double offset = isBlue ? (angleToGoal - Math.PI / 4) * offsetFactor : (angleToGoal + Math.PI / 4) * offsetFactor;
        double offset;
        if (dist > 0) {
            offset = isBlue ? ((angleToGoal - Math.PI / 4) / dist) * offsetFactor : ((angleToGoal + Math.PI / 4) / dist) * offsetFactor;
        } else {
            // in case pinpoint gives a weird coord in div 0 err
            offset = isBlue ? ((angleToGoal - Math.PI / 4)) * offsetFactor : ((angleToGoal + Math.PI / 4)) * offsetFactor;
        }

        // double offset = isBlue ? (angleToGoal - Math.PI / 4) * offsetFactor : (angleToGoal + Math.PI / 4) * offsetFactor;
        // when angle is big, aim more left (which is positive direction), when it is small, aim more right (negative direction)
        // opposite for red, and all this helps i guess? backboard area is better when we higher so it makes sense idk
        // what do we count as 0? i think we count it as the 45 degree position, which i suppose is
        // for red it is from the negative 45 i think, and let's offset everything my like 6-7% as a test

        double azimuth = Math.atan2(-(dx-vTangential.getXComponent()*timestep), (dy-vTangential.getYComponent()*timestep)) - robotPose.getHeading() + Math.toRadians(90) + offset;

//        double newX = robotPose.getX()-timestep*u.getXComponent();
//        double newY = robotPose.getY()-timestep*u.getYComponent();
//
//        Pose nextPos = new Pose(newX, newY, robotPose.getHeading());
//
//        double dxN = goal.getX() - nextPos.getX();
//        double dyN = goal.getY() - nextPos.getY();
//        double distN = Math.hypot(dxN, dyN);
//
//        double azimuthN = Math.atan2(-dxN, dyN) - robotPose.getHeading() + Math.toRadians(90) + offset;
//        double thetaN = thetaLUT.getValue(distN);
//        double velocityN = velocityLUT.getValue(distN);

        // return new double[] {azimuthN, thetaN, velocityN};

        return new double[] {azimuth, theta, velocity};
    }

    public double[] calculateAzimuthThetaVelocity(Pose robotPose, Vector robotVelocity, double currentShooterVelocity) {
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
        double thetaOut;
        double velocity = velocityLUT.getValue(dist);
        double theta = thetaLUT.getValue(dist) + Math.toRadians(34);
        double wheelVelocityInches = calculateLinearVelocityInches(velocityLUT.getValue(dist)) * Math.cos(theta);

        // check if in the domain of arccos so we don't error.
        // update the angle based on velocity to goal. hopefully it works.

        // on some intervals of velocity, adjusting the angle is not enough
        // TODO: NEED TO UPDATE NEW HOOD ANGLES. THE RANGE IS DIFFERENT NOW
        // the min angle is 34, the max angle is 49 -> 0 to 15
        // logic: if thetaOut < 0: solve for thetaOut = 0 (34 deg), and find a target wheelVelocityInches, convert to ticks

        // if its less than 0 then that means that velocity is too high
        // if it greater than 49 then velocity is too low.

        // it is a function of multiple variables: theta and wheel velocity. don't think this is really easily solvable
        // lowkey just use for loop, plug in, keep constraints. loop every like 10 ticks or something


        if (Math.abs(Math.cos(theta) + radialVelocityScaleFactor * velToGoal/wheelVelocityInches) < 1) {
            thetaOut = Math.acos(Math.cos(theta) + (radialVelocityScaleFactor * velToGoal/wheelVelocityInches)) - Math.toRadians(34);
            // if its less than 0 then that means that velocity is too high
            if (thetaOut < 0) {
                for (int i = 0; i < MAX_ITERATIONS; i++) {
                    double wvIn = calculateLinearVelocityInches(velocityLUT.getValue(dist) - 10 * i) * Math.cos(theta);
                    thetaOut = Math.acos(Math.cos(theta) + (radialVelocityScaleFactor * velToGoal/wvIn)) - Math.toRadians(34);
                    if (thetaOut > 0) {
                        velocity = calculateTicksPerSecond(wvIn) / Math.cos(theta);
                        break;
                    }
                    if (i > 65) {
                        // fallback
                        thetaOut = thetaLUT.getValue(dist);
                    }
                }
            } else if (thetaOut > 15) { // if it greater than 49 then velocity is too low.
                for (int i = 0; i < MAX_ITERATIONS; i++) {
                    double wvIn = calculateLinearVelocityInches(velocityLUT.getValue(dist) + 10 * i) * Math.cos(theta);
                    thetaOut = Math.acos(Math.cos(theta) + (radialVelocityScaleFactor * velToGoal/wvIn)) - Math.toRadians(34);
                    if (thetaOut < 15) {
                        velocity = calculateTicksPerSecond(wvIn) / Math.cos(theta);
                        break;
                    }
                    if (i > 65) {
                        // fallback
                        thetaOut = thetaLUT.getValue(dist);
                    }
                }
            }

        } else {
            thetaOut = thetaLUT.getValue(dist);
        }

        double timestep = constantTimeFactor + timeScaleFactor * (dist / (calculateLinearVelocityInches(velocityLUT.getValue(dist)) * Math.cos(thetaLUT.getValue(dist)+Math.toRadians(34))));

        // blue perspective:
        // pure angle to goal. from small angles, it overshoots to the left (from blue perspective this is positive turret),
        double angleToGoal = Math.atan2(-(dx-vTangential.getXComponent()*timestep), (dy-vTangential.getYComponent()*timestep));

        // TODO: idea: offset is a bigger problem at far distances (more emphasized), what if we divide offset by distance at the end?
        // this would make more sense... given that it is based on arc length
        double offset;
        if (dist > 0) {
            offset = isBlue ? ((angleToGoal - Math.PI / 4) / dist) * offsetFactor : ((angleToGoal + Math.PI / 4) / dist) * offsetFactor;
        } else {
            // in case pinpoint gives a weird coord in div 0 err
            offset = isBlue ? ((angleToGoal - Math.PI / 4)) * offsetFactor : ((angleToGoal + Math.PI / 4)) * offsetFactor;
        }


        double azimuth = Math.atan2(-(dx-vTangential.getXComponent()*timestep), (dy-vTangential.getYComponent()*timestep)) - robotPose.getHeading() + Math.toRadians(90) + offset;

        return new double[] {azimuth, thetaOut, velocity};
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
    // new idea:
    // we know that velocity cannot change much. have it be a very high number or something.
    // find an min angle, theta, where the ball can reach the target AND the y-value is high enough to be above the classifier
    // we start with a less than ideal velocity, and find a better one i guess

    // we will call simulateProjectileTOF() with the same velocity. except dt will be 0.01 for better computation.
    // loop through the nearest 15 degrees of angle
    // find the lowest that works.

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

    public void setTimeScaleFactor(double x) {
        timeScaleFactor = x;
    }

    public void setConstantTimeFactor(double x) {
        timeScaleFactor = x;
    }
    public void setOffsetFactor(double x) {offsetFactor=x;}

    // returns the amount of time a projectile will take in air. not yet implemented but
    // should be better than current sotm.
    // note that a timeconstant (like 0.1s) is needed b/c turret doesn't update instantly to target, it lags a bit,
    // so a larger target is always necessary
    // however there shouldn't need to be a scaling constant for this new one, if there is, it is small
    private double simulateProjectileTOF(double dist, double theta, double velocityTicks) {
        double m = 0.07845; // mass of ball in kg
        double v = calculateLinearVelocityMeters(velocityTicks); // ticks -> linear velocity
        double c = 0.5 * 1 * 1.225 * 0.01216604657; // 1/2 Cd * rho * cross sectional area in m^2

        theta += Math.toRadians(28); // because of weird offset trust
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
