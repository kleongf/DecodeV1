package org.firstinspires.ftc.teamcode.util.purepursuit;


import org.firstinspires.ftc.teamcode.util.controllers.HeadingPIDFController;
import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;
import static org.firstinspires.ftc.teamcode.util.purepursuit.FollowerConstants.*;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class PurePursuitFollower {
    private Localizer localizer;
    private Pose2D currentPose;
    private Pose2D goalPose;
    private double speed;
    private boolean isHoldingPoint = false;
    private boolean isFollowingPath = false;
    private boolean isP2Ping = false;
    private double holdPointScaleFactor;
    private double speedConstraint;
    private double headingConstraint;
    private double pathEndDistance;
    private double distanceConstraint;
    private double holdPointRange;

    // TODO: could make two different PID controllers, one for p2p for high velocities and one for normal p2p.
    private PIDFController longitudinalController;
    private PIDFController lateralController;
    private PIDFController headingController;

    private Path2D currentPath;
    private int currentPathIndex;
    private int lastFoundIndex;

    private double minLookAhead;
    private double maxLookAhead;
    private double KpHeading;
    private double KpSpeed;
    private double KpLookAhead;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearLeft;
    private DcMotorEx rearRight;

    public PurePursuitFollower(HardwareMap hardwareMap) {
        this.localizer = new Localizer(hardwareMap);
        this.currentPose = new Pose2D(0, 0, 0);
        this.longitudinalController = new PIDFController(LONGITUDINAL_COEFFICIENTS.kp, LONGITUDINAL_COEFFICIENTS.ki, LONGITUDINAL_COEFFICIENTS.kd, LONGITUDINAL_COEFFICIENTS.kf);
        this.lateralController = new PIDFController(LATERAL_COEFFICIENTS.kp, LATERAL_COEFFICIENTS.ki, LATERAL_COEFFICIENTS.kd, LATERAL_COEFFICIENTS.kf);
        this.headingController = new PIDFController(HEADING_COEFFICIENTS.kp, HEADING_COEFFICIENTS.ki, HEADING_COEFFICIENTS.kd, HEADING_COEFFICIENTS.kf);
        this.holdPointScaleFactor = HOLD_POINT_SCALE_FACTOR;
        this.holdPointRange = HOLD_POINT_DISTANCE;
        this.speedConstraint = PATH_END_SPEED_CONSTRAINT;
        this.headingConstraint = PATH_END_HEADING_CONSTRAINT;
        this.distanceConstraint = PATH_END_DISTANCE_CONSTRAINT;
        this.minLookAhead = LOOK_AHEAD_MIN_DISTANCE;
        this.maxLookAhead = LOOK_AHEAD_MAX_DISTANCE;
        this.KpLookAhead = KP_LOOK_AHEAD;
        this.KpSpeed = KP_SPEED;
        this.KpHeading = KP_HEADING;
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
    }

    public void setStartingPose(Pose startPose) {
        currentPose = new Pose2D(startPose.getX(), startPose.getY(), startPose.getHeading());
        localizer.setStartPose(startPose);
    }

    private double calculateLookAheadDistance() {
        return Math.min(minLookAhead + KpLookAhead * speed, maxLookAhead);
    }

    private void calculateGoalPose() {
        double lookAheadDistance = calculateLookAheadDistance();
        double posX = currentPose.getX();
        double posY = currentPose.getY();
        Pose2D goal = (lastFoundIndex == currentPath.getSize()-1) ? currentPath.getPose(lastFoundIndex) : currentPath.getPose(lastFoundIndex+1);

        for (int i = currentPathIndex; i < currentPath.getSize() - 1; i++) {
            double x1 = currentPath.getPose(i).getX() - posX;
            double y1 = currentPath.getPose(i).getY() - posY;
            double x2 = currentPath.getPose(i + 1).getX() - posX;
            double y2 = currentPath.getPose(i + 1).getY() - posY;
            double dx = x2 - x1;
            double dy = y2 - y1;
            double dr = Math.sqrt(dx * dx + dy * dy);
            double det = x1 * y2 - x2 * y1;
            double discriminant = (lookAheadDistance * lookAheadDistance) * (dr * dr) - (det * det);

            if (discriminant >= 0) {
                double sol_x1 = (det * dy + Math.signum(dy) * dx * Math.sqrt(discriminant)) / (dr * dr);
                double sol_x2 = (det * dy - Math.signum(dy) * dx * Math.sqrt(discriminant)) / (dr * dr);
                double sol_y1 = (-det * dx + Math.abs(dy) * Math.sqrt(discriminant)) / (dr * dr);
                double sol_y2 = (-det * dx - Math.abs(dy) * Math.sqrt(discriminant)) / (dr * dr);

                Pose2D sol1 = new Pose2D(sol_x1 + posX, sol_y1 + posY, 0);
                Pose2D sol2 = new Pose2D(sol_x2 + posX, sol_y2 + posY, 0);

                double minX = Math.min(currentPath.getPose(i).getX(), currentPath.getPose(i + 1).getX());
                double maxX = Math.max(currentPath.getPose(i).getX(), currentPath.getPose(i + 1).getX());
                double minY = Math.min(currentPath.getPose(i).getY(), currentPath.getPose(i + 1).getY());
                double maxY = Math.max(currentPath.getPose(i).getY(), currentPath.getPose(i + 1).getY());

                if (((sol1.getX() >= minX && sol1.getX() <= maxX) && (sol1.getY() >= minY && sol1.getY() <= maxY)) || ((sol2.getX() >= minX && sol2.getX() <= maxX) && (sol2.getY() >= minY && sol2.getY() <= maxY))) {
                    if (((sol1.getX() >= minX && sol1.getX() <= maxX) && (sol1.getY() >= minY && sol1.getY() <= maxY)) && ((sol2.getX() >= minX && sol2.getX() <= maxX) && (sol2.getY() >= minY && sol2.getY() <= maxY))) {
                        if (MathFunctions.getDistance(currentPath.getPose(i + 1), sol1) < MathFunctions.getDistance(currentPath.getPose(i + 1), sol2)) {
                            goal = sol1;
                        } else {
                            goal = sol2;
                        }
                    } else {
                        if ((sol1.getX() >= minX && sol1.getX() <= maxX) && (sol1.getY() >= minY && sol1.getY() <= maxY)) {
                            goal = sol1;
                        } else {
                            goal = sol2;
                        }
                    }

                    if (MathFunctions.getDistance(goal, currentPath.getPose(i + 1)) < MathFunctions.getDistance(currentPose, currentPath.getPose(i+1))) {
                        lastFoundIndex = i;
                        break;
                    } else {
                        lastFoundIndex = i + 1;
                        break;
                    }
                }
            }
        }
        goalPose = goal;
        currentPathIndex = lastFoundIndex;
    }

    public void setMotorPowers(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(backRightPower);
    }
    private void setFollowerMotorPowers(double yDist, double hError) {
        // new plan: our setpoint is max velocity
        // however when we reach the end of our path we set to 0 (based on acceleration we can compute)
        // v^2 = vo^2 + 2adx
        // so dx = vo^2/2a
        // none of this code does anything
        double targetSpeed = MAX_VELOCITY;
        double distanceToEnd = (speed * speed) / (2 * MAX_ACCELERATION);
        if ((MathFunctions.getDistance(currentPose, currentPath.getPose(currentPathIndex)) < distanceToEnd) && (currentPathIndex == currentPath.getSize() - 1)) {
            targetSpeed = 0;
        }
        double outputSpeed = KP_SPEED * Math.signum(yDist) * (targetSpeed - speed) + KV_SPEED * Math.signum(yDist) * (targetSpeed);
        // im just gonna set them equal for now, we know max value is likely max velocity, which we want to go at full speed, so it should be 0.02?
        double outputHeading = KP_HEADING * hError;
        // we probably wont be making more than 90 degree turns so i will base it off that: pi/2 is about 3/2, so wrapping it to 1 we have 0.66 -> 0.7
        // i want robot to prioritize turning
        // i need a function that can map pi (max error) to 1 (all turning) to just addition
        // so as heading error -> 0 then the horizontal multiplier -> 1
        // but i need a better function. maybe cos(err/2)? if error = pi than it is 0, but if error = pi/8 or so then it is 0.5 or 1/3 of the power
        // lets just use this for now: a linear scaling factor that equals 0 when it is pi and 1 when it is 0
        // method 1
        setMotorPowers(0, ((Math.PI-Math.abs(hError))/Math.PI) * MathFunctions.clamp(outputSpeed, -1, 1), MathFunctions.clamp(outputHeading, -1, 1));

        // method 2:
        // wait till heading error < 10 degrees
//        if (Math.abs(hError) < Math.toRadians(10)) {
//            setMotorPowers(0, MathFunctions.clamp(outputSpeed, -1, 1), MathFunctions.clamp(outputHeading, -1, 1));
//        } else {
//            setMotorPowers(0, 0, MathFunctions.clamp(outputHeading, -1, 1));
//        }

        // method 3: do nothing
        //setMotorPowers(0, MathFunctions.clamp(outputSpeed, -1, 1), MathFunctions.clamp(outputHeading, -1, 1));
    }

    private void setP2PMotorPowers(double scaleFactor) {
        // i think the problem is that we aren't using math cos and math sin
        // let me fix this

        // 1. gpt code gonna help me out lol
        double outX = lateralController.calculate(currentPose.getX(), goalPose.getX());
        double outY = longitudinalController.calculate(currentPose.getY(), goalPose.getY());
        double outHeading = -headingController.calculate(MathFunctions.angleWrap(currentPose.getHeading()), MathFunctions.angleWrap(goalPose.getHeading()));
        // double outHeading = headingController.calculate(currentPose.getHeading(), goalPose.getHeading());

        // 4) Convert GLOBAL outputs to ROBOT-LOCAL frame using current heading:
        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());

        // Global velocity-like outputs -> local robot frame
        // [vx_local]   [ cos  sin ] [outX]
        // [vy_local] = [-sin  cos ] [outY]
        double xPower =  outX * cosH + outY * sinH;
        double yPower = -outX * sinH + outY * cosH;
//        double xPower = 0;
//        double yPower = 0;
        double headingPower = outHeading; // rotation is already body-centric sign

        // double xPower = longitudinalController.calculate(Math.sin(currentPose.getHeading()) * currentPose.getX(), Math.sin(goalPose.getHeading()) * goalPose.getX());
        System.out.println("X Power: " + xPower);
        // double yPower = lateralController.calculate(Math.cos(currentPose.getHeading()) * currentPose.getY(), Math.cos(goalPose.getHeading()) * goalPose.getY());
        System.out.println("Y Power: " + yPower);
        // double headingPower = headingController.calculate(MathFunctions.angleWrap(goalPose.getHeading()-currentPose.getHeading()));
        System.out.println("Heading Power: " + headingPower);
        System.out.println("Goal Heading: " + goalPose.getHeading());

        setMotorPowers(scaleFactor * xPower, scaleFactor * yPower, scaleFactor * headingPower);
    }

    public void followPath(Path2D path) {
        currentPath = path;
        isFollowingPath = true;
        isHoldingPoint = false;
        isP2Ping = false;
        currentPathIndex = 0;
        lastFoundIndex = 0;
    }
    public void p2p(Pose2D pose) {
        goalPose = pose;
        isFollowingPath = false;
        isHoldingPoint = false;
        isP2Ping = true;
    }

    public void update() {
        localizer.update();
        currentPose = localizer.getPose2D();
        speed = localizer.getSpeed();
        System.out.println("Speed: " + speed);
        System.out.println("Pose: X: " + currentPose.getX() + " Y: " + currentPose.getY());
        System.out.println("Heading: " + currentPose.getHeading());
        double distanceToEnd = (speed * speed) / (2 * MAX_ACCELERATION);
        if (currentPath != null) {
            System.out.println("Distance To Target: " + MathFunctions.getDistance(currentPose, currentPath.getPose(currentPathIndex)));
            System.out.println("End of current path: " + (currentPathIndex == currentPath.getSize() - 1));
            if ((MathFunctions.getDistance(currentPose, currentPath.getPose(currentPathIndex)) < distanceToEnd) && (currentPathIndex == currentPath.getSize() - 1)) {
                System.out.println("SPEED TARGET IS 0");
            }
        }


        System.out.println("Distance TO End: " + distanceToEnd);
        if (isP2Ping) {
            System.out.println("P2PING");
        }

        if (isFollowingPath) {
            // TODO: done maybe we should put the p2p function in here? if it's past the deceleration constraint then p2p, it may not be a good idea to do based off of distance
            //double distanceToEnd = (speed * speed) / (2 * MAX_ACCELERATION);
//            if ((MathFunctions.getDistance(currentPose, currentPath.getPose(currentPathIndex)) < distanceToEnd) && (currentPathIndex == currentPath.getSize() - 1)) {
//                goalPose = currentPath.getPose(currentPath.getSize() - 1);
//                setP2PMotorPowers(1.0);
//                isP2Ping = true;
//                isFollowingPath = false;
//                isHoldingPoint = false;
//                currentPath = null;
//                currentPathIndex = 0;
//                lastFoundIndex = 0;
//                return;
//            }
            // i just realized these are the same lol
            // not distanceconstraint anymore
            // TODO: might want to not restrict this to the final path, and let it work for any point on the path. We could set acceleration to 180 like it was, but it could p2p at 60^2/(2 * 180) or 10 inches.
            if ((MathFunctions.getDistance(currentPose, currentPath.getPose(currentPath.getSize()-1)) < distanceToEnd)) {
//            if ((MathFunctions.getDistance(currentPose, currentPath.getPose(currentPathIndex)) < distanceToEnd) && (currentPathIndex == currentPath.getSize() - 1)) {
                goalPose = currentPath.getPose(currentPath.getSize() - 1);
                setP2PMotorPowers(1.0);
                isP2Ping = true;
                isFollowingPath = false;
                isHoldingPoint = false;
                currentPath = null;
                currentPathIndex = 0;
                lastFoundIndex = 0;
                return;
            } else {
                calculateGoalPose();
                double angleToGoal = Math.atan2(goalPose.getY()-currentPose.getY(), goalPose.getX()-currentPose.getX());
                double dist = MathFunctions.getDistance(currentPose, goalPose);
                if (currentPath.isReversed()) {
                    angleToGoal -= Math.PI;
                    dist *= -1;
                }
                double angleDiff = -MathFunctions.angleWrap(angleToGoal - currentPose.getHeading());
                // so i think the angle thing is fine?
                setFollowerMotorPowers(dist, angleDiff);
            }
            return;
        }
        if (isP2Ping) {
            if (MathFunctions.getDistance(currentPose, goalPose) < holdPointRange && speed < speedConstraint && Math.abs(MathFunctions.angleWrap(goalPose.getHeading()-currentPose.getHeading())) < headingConstraint) {
                isP2Ping = false;
                isFollowingPath = false;
                isHoldingPoint = true;
                return;
            }
            setP2PMotorPowers(1.0);
            return;
        }
        if (isHoldingPoint) {
            System.out.println("Holding Point");
            setP2PMotorPowers(holdPointScaleFactor);
        }
    }

    public boolean isFinished() {
        return !isP2Ping && !isFollowingPath;
    }

    // we keep following paths i guess. Once the last point in a "path" is reached, we will pid to point
}
