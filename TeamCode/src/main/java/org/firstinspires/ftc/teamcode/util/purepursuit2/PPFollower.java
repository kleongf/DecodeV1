package org.firstinspires.ftc.teamcode.util.purepursuit2;

import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.FRICTION_CONSTANT;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.HEADING_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.HOLD_POINT_SCALE_FACTOR;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.KQ_X;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.KQ_Y;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.LATERAL_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.LONGITUDINAL_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.LOOK_AHEAD_DISTANCE;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.MAX_ACCELERATION;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.NOMINAL_VOLTAGE;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.PATH_END_DISTANCE_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.VOLTAGE_COMP_AUTO;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.X_ZPA;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.Y_ZPA;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.leftFrontMotorDirection;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.leftRearMotorDirection;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.rightFrontMotorDirection;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.rightRearMotorDirection;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.rightRearMotorName;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.KA_X;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.KA_Y;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.KS_X;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.KS_Y;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.KV_HEADING;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.KV_X;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.KV_Y;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.MAX_VELOCITY;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.PATH_END_HEADING_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.PATH_END_SPEED_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.PID_MAX_VELOCITY;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;


public class PPFollower {
    private enum PPState {
        IDLE,
        HOLDING_POINT,
        PID_TO_POINT,
        FOLLOWING_PATH
    }
    private PPState state = PPState.IDLE;
    private PPLocalizer localizer;
    public Pose currentPose;
    private Pose goalPose;
    private PPPath currentPath;
    private int currentPathIndex;
    private int lastFoundIndex;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearLeft;
    private DcMotorEx rearRight;
    private VoltageSensor voltageSensor;
    private double lookAheadDistance;
    private double maxVelocity;
    private double maxAcceleration;
    private double maxPower;
    private double pathEndDistanceConstraint;
    private double pathEndHeadingConstraint;
    private double pathEndSpeedConstraint;
    private boolean holdPoint;
    private double holdPointScaleFactor;
    private double lastTimeStamp = 0;
    private double kp_x = 0.06;
    private double kd_x = 0.003;
    private double kp_y = 0.03;
    private double kd_y = 0.0015;
    private double kp_heading = 1.5;
    private double kd_heading = 0.03;
    private double lastHeadingError = 0;
    private double lastXError = 0;
    private double lastYError = 0;
    private double period = 0.03;
    private double kS = 0.1;

    public PPFollower(HardwareMap hardwareMap) {
        this.localizer = new PPLocalizer(hardwareMap);
        this.currentPose = new Pose(0, 0, 0);
        this.currentPathIndex = 0;

        this.frontLeft = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        this.rearLeft = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        this.frontRight = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        this.rearRight = hardwareMap.get(DcMotorEx.class, rightRearMotorName);

        this.frontLeft.setDirection(leftFrontMotorDirection);
        this.rearLeft.setDirection(leftRearMotorDirection);
        this.frontRight.setDirection(rightFrontMotorDirection);
        this.rearRight.setDirection(rightRearMotorDirection);

        this.frontLeft.setDirection(leftFrontMotorDirection);
        this.rearLeft.setDirection(leftRearMotorDirection);
        this.frontRight.setDirection(rightFrontMotorDirection);
        this.rearRight.setDirection(rightRearMotorDirection);

        this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.lookAheadDistance = LOOK_AHEAD_DISTANCE;
        this.maxVelocity = MAX_VELOCITY;
        this.maxAcceleration = MAX_ACCELERATION;
        this.pathEndDistanceConstraint = PATH_END_DISTANCE_CONSTRAINT;
        this.pathEndSpeedConstraint = PATH_END_SPEED_CONSTRAINT;
        this.pathEndHeadingConstraint = PATH_END_HEADING_CONSTRAINT;
        this.holdPointScaleFactor = HOLD_POINT_SCALE_FACTOR;
        this.holdPoint = true;
        this.maxPower = 1.0;

        this.voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    public void setStartingPose(Pose startPose) {
        currentPose = startPose;
        localizer.setStartPose(startPose);
    }

    private double getVoltageScaler() {
        return (NOMINAL_VOLTAGE - (NOMINAL_VOLTAGE * FRICTION_CONSTANT)) / (voltageSensor.getVoltage() - ((Math.pow(NOMINAL_VOLTAGE, 2) / voltageSensor.getVoltage()) * FRICTION_CONSTANT));
    }
    // this is a big hot mess and it WORKS so i'm not gonna touch it
//    private void calculateGoalPose() {
//        double posX = currentPose.getX();
//        double posY = currentPose.getY();
//        Pose goal = (lastFoundIndex == currentPath.getSize()-1) ? currentPath.getPose(lastFoundIndex) : currentPath.getPose(lastFoundIndex+1);
//
//        for (int i = currentPathIndex; i < currentPath.getSize() - 1; i++) {
//            double x1 = currentPath.getPose(i).getX() - posX;
//            double y1 = currentPath.getPose(i).getY() - posY;
//            double x2 = currentPath.getPose(i + 1).getX() - posX;
//            double y2 = currentPath.getPose(i + 1).getY() - posY;
//            double dx = x2 - x1;
//            double dy = y2 - y1;
//            double dr = Math.sqrt(dx * dx + dy * dy);
//            double det = x1 * y2 - x2 * y1;
//            double discriminant = (lookAheadDistance * lookAheadDistance) * (dr * dr) - (det * det);
//
//            if (discriminant >= 0) {
//                double sol_x1 = (det * dy + Math.signum(dy) * dx * Math.sqrt(discriminant)) / (dr * dr);
//                double sol_x2 = (det * dy - Math.signum(dy) * dx * Math.sqrt(discriminant)) / (dr * dr);
//                double sol_y1 = (-det * dx + Math.abs(dy) * Math.sqrt(discriminant)) / (dr * dr);
//                double sol_y2 = (-det * dx - Math.abs(dy) * Math.sqrt(discriminant)) / (dr * dr);
//
//                Pose sol1 = new Pose(sol_x1 + posX, sol_y1 + posY, 0);
//                Pose sol2 = new Pose(sol_x2 + posX, sol_y2 + posY, 0);
//                // if it's not tangent we try to go to its normal heading
//                if (!currentPath.isTangent()) {
//                    sol1.setHeading(currentPath.getPose(i).getHeading());
//                    sol2.setHeading(currentPath.getPose(i).getHeading());
//                } else {
//                    sol1.setHeading(Math.atan2(sol1.getY()-currentPose.getY(), sol1.getX()-currentPose.getX()));
//                    sol2.setHeading(Math.atan2(sol2.getY()-currentPose.getY(), sol2.getX()-currentPose.getX()));
//                }
//
//                double minX = Math.min(currentPath.getPose(i).getX(), currentPath.getPose(i + 1).getX());
//                double maxX = Math.max(currentPath.getPose(i).getX(), currentPath.getPose(i + 1).getX());
//                double minY = Math.min(currentPath.getPose(i).getY(), currentPath.getPose(i + 1).getY());
//                double maxY = Math.max(currentPath.getPose(i).getY(), currentPath.getPose(i + 1).getY());
//
//                if (((sol1.getX() >= minX && sol1.getX() <= maxX) && (sol1.getY() >= minY && sol1.getY() <= maxY)) || ((sol2.getX() >= minX && sol2.getX() <= maxX) && (sol2.getY() >= minY && sol2.getY() <= maxY))) {
//                    if (((sol1.getX() >= minX && sol1.getX() <= maxX) && (sol1.getY() >= minY && sol1.getY() <= maxY)) && ((sol2.getX() >= minX && sol2.getX() <= maxX) && (sol2.getY() >= minY && sol2.getY() <= maxY))) {
//                        if (MathUtil.distance(currentPath.getPose(i + 1), sol1) < MathUtil.distance(currentPath.getPose(i + 1), sol2)) {
//                            goal = sol1;
//                        } else {
//                            goal = sol2;
//                        }
//                    } else {
//                        if ((sol1.getX() >= minX && sol1.getX() <= maxX) && (sol1.getY() >= minY && sol1.getY() <= maxY)) {
//                            goal = sol1;
//                        } else {
//                            goal = sol2;
//                        }
//                    }
//
//                    if (MathUtil.distance(goal, currentPath.getPose(i + 1)) < MathUtil.distance(currentPose, currentPath.getPose(i+1))) {
//                        lastFoundIndex = i;
//                        break;
//                    } else {
//                        lastFoundIndex = i + 1;
//                        break;
//                    }
//                }
//            }
//        }
//        goalPose = goal;
//        currentPathIndex = lastFoundIndex;
//    }
    private void calculateGoalPose() {
        double rx = currentPose.getX();
        double ry = currentPose.getY();

        Pose goal = currentPath.getPose(currentPath.getSize() - 1); // fallback

        int bestSeg = lastFoundIndex;
        double bestT = -1.0;
        Pose bestPoint = null;

        for (int i = lastFoundIndex; i < currentPath.getSize() - 1; i++) {
            Pose p1 = currentPath.getPose(i);
            Pose p2 = currentPath.getPose(i + 1);

            double x1 = p1.getX() - rx;
            double y1 = p1.getY() - ry;
            double x2 = p2.getX() - rx;
            double y2 = p2.getY() - ry;

            double dx = x2 - x1;
            double dy = y2 - y1;
            double a = dx * dx + dy * dy;
            if (a < 1e-6) continue;

            double b = 2 * (x1 * dx + y1 * dy);
            double c = x1 * x1 + y1 * y1 - lookAheadDistance * lookAheadDistance;

            double disc = b * b - 4 * a * c;
            if (disc < 0) continue;

            double sqrtDisc = Math.sqrt(Math.max(0, disc));
            double t1 = (-b - sqrtDisc) / (2 * a);
            double t2 = (-b + sqrtDisc) / (2 * a);

            // Check both roots
            if (t1 >= 0 && t1 <= 1) {
                if (i > bestSeg || (i == bestSeg && t1 > bestT)) {
                    double gx = p1.getX() + t1 * (p2.getX() - p1.getX());
                    double gy = p1.getY() + t1 * (p2.getY() - p1.getY());
                    bestPoint = new Pose(gx, gy, 0);
                    bestSeg = i;
                    bestT = t1;
                }
            }

            if (t2 >= 0 && t2 <= 1) {
                if (i > bestSeg || (i == bestSeg && t2 > bestT)) {
                    double gx = p1.getX() + t2 * (p2.getX() - p1.getX());
                    double gy = p1.getY() + t2 * (p2.getY() - p1.getY());
                    bestPoint = new Pose(gx, gy, 0);
                    bestSeg = i;
                    bestT = t2;
                }
            }
        }

        if (bestPoint != null) {
            double heading;
            if (!currentPath.isTangent()) {
                heading = currentPath.getPose(bestSeg).getHeading();
            } else {
                Pose next = currentPath.getPose(bestSeg + 1);
                heading = Math.atan2(next.getY() - bestPoint.getY(),
                        next.getX() - bestPoint.getX());
            }

            bestPoint.setHeading(heading);
            goal = bestPoint;
            lastFoundIndex = bestSeg;
        }

        goalPose = goal;
    }





    // From WPILib (and some paper): For a DC Motor, V = kS * sgn(x) + kV * x' + kA * x''
    // kS: oppose friction
    // kV: target velocity
    // kA: target acceleration

    // moves in the general direction of the goal pose.
    private void moveToPose(Pose pose) {
        double dy = pose.getY()-currentPose.getY();
        double dx = pose.getX()-currentPose.getX();
        double dTheta = MathUtil.normalizeAngle(pose.getHeading()-currentPose.getHeading());

        double kY = 0.5; // doesnt matter but this is a better number ig.
        double kX = 1; // takes more power to move x, about double i think
        double kTheta = 0.5/Math.PI;

        // 4) Convert GLOBAL outputs to ROBOT-LOCAL frame using current heading:
        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());

        double xPower =  (sinH * dx  -  cosH * dy) * kX;
        double yPower =  (cosH * dx  +  sinH * dy) * kY;
        double thetaPower = kTheta * dTheta; // rotation is already body-centric sign

        // now make sure that x and y add up to 1
        double totalXY = Math.abs(xPower) + Math.abs(yPower);
        xPower /= totalXY;
        yPower /= totalXY;

        // now make it so they add up to 1/2. could have done this earlier but whatever
        xPower /= 2;
        yPower /= 2;

        // require 1 power at all times
        double total = Math.abs(xPower) + Math.abs(yPower) + Math.abs(thetaPower);
        xPower /= total;
        yPower /= total;
        thetaPower /= total;

        setMotorPowers(xPower, yPower, -thetaPower);
    }

    private void setMotorPowers(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        if (VOLTAGE_COMP_AUTO) {
            double scaler = getVoltageScaler();
            frontLeftPower *= scaler;
            backLeftPower *= scaler;
            frontRightPower *= scaler;
            backRightPower *= scaler;
        }

        frontLeft.setPower(MathUtil.clamp(frontLeftPower, -1, 1));
        rearLeft.setPower(MathUtil.clamp(backLeftPower, -1, 1));
        frontRight.setPower(MathUtil.clamp(frontRightPower, -1, 1));
        rearRight.setPower(MathUtil.clamp(backRightPower, -1, 1));
    }

    private void PIDToPose(double scaleFactor) {
        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());

        double errorHeading = MathUtil.normalizeAngle(goalPose.getHeading()-currentPose.getHeading());
        double errorX = goalPose.getX()-currentPose.getX();
        double errorY = goalPose.getY()-currentPose.getY();

        Matrix C = new Matrix(new double[][]{
                {sinH, -cosH, 0},
                {cosH,  sinH, 0},
                {0,     0,    1},
        });

        Matrix X = new Matrix(new double[][]{
                {errorX, errorY, errorHeading}
        }).transpose();

        Matrix B = C.multiply(X);

        double robotErrX = B.get(0, 0);
        double robotErrY = B.get(1, 0);
        double robotErrHeading = B.get(2, 0);

        double xPower =  kp_x * robotErrX + kd_x * (robotErrX - lastXError) / period;
        double yPower =  kp_y * robotErrX + kd_y * (robotErrY - lastYError) / period;
        double headingPower = kp_heading * robotErrHeading + kd_heading * (robotErrHeading-lastHeadingError) / period;

        // a small thing: calculate robot velocity, add on extra scaling power to brake if necessary
//        Matrix V = new Matrix(new double[][]{
//                {localizer.getVelocity().getX(), localizer.getVelocity().getY(), 0}
//        }).transpose();
//
//        Matrix V2 = C.multiply(V);
//
//        double xVel = V2.get(0, 0);
//        double yVel = V2.get(1, 0);
//
//        // calculate dist to end as speed^2 / 2 * zpam
//        // if this dist is less than dist to end then no ff is needed, we can basically coast to the end.
//        double xDistZPA = (xVel * xVel) / (-2 * X_ZPA);
//        double yDistZPA = (yVel * yVel) / (-2 * Y_ZPA);
//
//        double kBrakeX = 0.001; // it is harder for x to brake
//        double kBrakeY = 0.0005; // easier for y to brake
//
//        // if distance is too large for zpa, then continue applying quadratic braking
//        if (robotErrX > xDistZPA) {
//            xPower += kBrakeX * -Math.abs(xVel); // brake, so that we go in opposite direction
//        }
//
//        if (robotErrY > yDistZPA) {
//            yPower += kBrakeY * -Math.abs(yVel); // brake, so that we go in opposite direction
//        }
        // for now i will just add the braking coefficient if the velocity is too high
        Matrix V = new Matrix(new double[][]{
                {localizer.getVelocity().getX(), localizer.getVelocity().getY(), 0}
        }).transpose();

        Matrix V2 = C.multiply(V);

        double xVel = V2.get(0, 0);
        double yVel = V2.get(1, 0);


        double kBrakeX = 0.0001; // it is harder for x to brake
        double kBrakeY = 0.00005; // easier for y to brake
        double maxVX = 10;
        double maxVY = 20;

        // if distance is too large for zpa, then continue applying quadratic braking
//        if (Math.abs(xVel) > maxVX) {
//            xPower += kBrakeX * -Math.abs(xVel) * xVel; // brake, so that we go in opposite direction
//        }
        // turned this off for now
        xPower += kBrakeX * -Math.abs(xVel) * xVel; // brake, so that we go in opposite direction
        yPower += kBrakeY * -Math.abs(yVel) * yVel; // brake, so that we go in opposite direction

//        if (Math.abs(yVel) > maxVY) {
//            yPower += kBrakeY * -Math.abs(yVel) * yVel; // brake, so that we go in opposite direction
//        }

        double total = Math.abs(xPower) + Math.abs(yPower) + Math.abs(headingPower);

        if (total > 1) {
            xPower /= total;
            yPower /= total;
            headingPower /= total;
        }
        xPower += kS * Math.signum(xPower);
        yPower += kS * Math.signum(yPower);

        lastXError = robotErrX;
        lastYError = robotErrY;
        lastHeadingError = robotErrHeading;

        setMotorPowers(scaleFactor * xPower, scaleFactor * yPower, scaleFactor * -headingPower);
    }

    public void followPath(PPPath path) {
        state = PPState.FOLLOWING_PATH;
        lookAheadDistance = path.getLookAheadDistance();
        maxVelocity = path.getMaxVelocity();
        maxAcceleration = path.getMaxAcceleration();
        pathEndSpeedConstraint = path.getPathEndSpeedConstraint();
        pathEndHeadingConstraint = path.getPathEndHeadingConstraint();
        pathEndDistanceConstraint = path.getPathEndDistanceConstraint();
        holdPointScaleFactor = path.getHoldPointScaleFactor();
        maxPower = path.getMaxPower();
        holdPoint = path.getHoldPoint();
        currentPath = path;
        currentPathIndex = 0;
        lastFoundIndex = 0;
    }

    public void update() {
        localizer.update();
        currentPose = localizer.getPose();
        double speed = localizer.getSpeed();
        // i don't like this method, because it's not guaranteed that the direction of the velocity vector
        // is the same direction as the path it needs to follow
        // we could project the velocity vector onto the vector of the path, take its magnitude.
        // double distanceToEnd = (speed * speed) / (2 * MAX_ACCELERATION);
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        switch (state) {
            case IDLE:
                break;
            case FOLLOWING_PATH:
                // System.out.println("FOLLOWING PATH");
                calculateGoalPose();
                // the second condition is a better catch, so that we don't go backwards from pure pursuit
                //Vector v = com.pedropathing.pathgen.MathFunctions.subtractVectors(goalPose.getVector(), currentPose.getVector());
                //Vector u = localizer.getVelocityVector();

                // (u ⋅ v / |v|²) * v
                // Vector projuv = com.pedropathing.pathgen.MathFunctions.scalarMultiplyVector(v, com.pedropathing.pathgen.MathFunctions.dotProduct(u, v) / com.pedropathing.pathgen.MathFunctions.dotProduct(v, v));

                // double velToGoal = projuv.getMagnitude();
                //double distanceToEnd = (velToGoal * velToGoal) / (2 * MAX_ACCELERATION);
                double distanceToEnd = (speed * speed) / (2 * MAX_ACCELERATION);

                if ((MathUtil.distance(currentPose, currentPath.getPose(currentPath.getSize()-1)) < distanceToEnd) || MathUtil.distance(currentPose, currentPath.getPose(currentPath.getSize()-1)) < lookAheadDistance) {
                    goalPose = currentPath.getPose(currentPath.getSize() - 1);
                    state = PPState.PID_TO_POINT;
                } else {
                    if (currentPath.isReversed() && currentPath.isTangent()) {
                        moveToPose(MathUtil.reverseHeading(goalPose));
                    } else {
                        moveToPose(goalPose);
                    }
                }
                break;
            case PID_TO_POINT:
//                System.out.println("PID TO POSE");
//                System.out.println("DISTANCE TO END: " + MathUtil.distance(currentPose, goalPose));
                if (MathUtil.distance(currentPose, goalPose) < pathEndDistanceConstraint && speed < pathEndSpeedConstraint && Math.abs(MathUtil.normalizeAngle(currentPose.getHeading()-goalPose.getHeading())) < pathEndHeadingConstraint) {
                    if (holdPoint) {
                        state = PPState.HOLDING_POINT;
                        holdPointScaleFactor = currentPath.getHoldPointScaleFactor();
                    } else {
                        breakFollowing();
                    }
                } else {
                    PIDToPose(maxPower);
                }
                break;
            case HOLDING_POINT:
                // resetting stuff
                lookAheadDistance = LOOK_AHEAD_DISTANCE;
                maxVelocity = MAX_VELOCITY;
                maxAcceleration = MAX_ACCELERATION;
                pathEndSpeedConstraint = PATH_END_SPEED_CONSTRAINT;
                pathEndHeadingConstraint = PATH_END_HEADING_CONSTRAINT;
                pathEndDistanceConstraint = PATH_END_DISTANCE_CONSTRAINT;
                maxPower = 1.0;
                holdPoint = true;
                currentPath = null;
                currentPathIndex = 0;
                lastFoundIndex = 0;
                PIDToPose(holdPointScaleFactor);
                break;
        }
    }

    public void breakFollowing() {
        state = PPState.IDLE;
        lookAheadDistance = LOOK_AHEAD_DISTANCE;
        maxVelocity = MAX_VELOCITY;
        maxAcceleration = MAX_ACCELERATION;
        pathEndSpeedConstraint = PATH_END_SPEED_CONSTRAINT;
        pathEndHeadingConstraint = PATH_END_HEADING_CONSTRAINT;
        pathEndDistanceConstraint = PATH_END_DISTANCE_CONSTRAINT;
        holdPointScaleFactor = HOLD_POINT_SCALE_FACTOR;
        maxPower = 1.0;
        holdPoint = true;
        currentPath = null;
        currentPathIndex = 0;
        lastFoundIndex = 0;
    }

    public boolean isBusy() {
        return (state == PPState.FOLLOWING_PATH || state == PPState.PID_TO_POINT);
    }

    public int getCurrentPathIndex() {
        return currentPathIndex;
    }

    public Vector getCurrentVelocity() {
        return getCurrentVelocity();
    }
}
