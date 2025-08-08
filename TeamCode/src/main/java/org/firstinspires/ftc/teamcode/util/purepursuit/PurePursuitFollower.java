package org.firstinspires.ftc.teamcode.util.purepursuit;


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

        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        Pose2D goal = currentPath.getPose(lastFoundIndex + 1);

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

                    if (MathFunctions.getDistance(goal, currentPath.getPose(i + 1)) < MathFunctions.getDistance(currentPose, currentPath.getPose(i + 1))) {
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

    private void setMotorPowers(double x, double y, double rx) {
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
        // nah use velocity control. i will use a p controller for now because lazy lol
        // TODO: nah i set them all to 1 because why not lol we gonna be fast af also lazy
        // WAIT THIS REALLY BAD WTF WAS I THINKGING
//        double normalizedSpeed = Math.signum(yDist);
//        double normalizedHeading = Math.signum(hError);
//        double normalizedSpeed = MathFunctions.clamp(KpSpeed * yDist, -1, 1);
//        double normalizedHeading = MathFunctions.clamp(KpHeading * hError, -1, 1);

        // new plan: our setpoint is max velocity
        // however when we reach the end of our path we set to 0 (based on acceleration we can compute)
        // v^2 = vo^2 + 2adx
        // so dx = vo^2/2a
        double targetSpeed = MAX_VELOCITY;
        double distanceToEnd = (speed * speed) / (2 * MAX_ACCELERATION);
        if ((MathFunctions.getDistance(currentPose, currentPath.getPose(currentPathIndex)) < distanceToEnd) && (currentPathIndex == currentPath.getSize() - 1)) {
            targetSpeed = 0;
        }
        double outputSpeed = KP_SPEED * Math.signum(yDist) * (targetSpeed - speed) + KV_SPEED * Math.signum(yDist) * (targetSpeed);
        // im just gonna set them equal for now, we know max value is likely max velocity, which we want to go at full speed, so it should be 0.02?
        double outputHeading = KP_HEADING * hError;
        // we probably wont be making more than 90 degree turns so i will base it off that: pi/2 is about 3/2, so wrapping it to 1 we have 0.66 -> 0.7

        setMotorPowers(0, MathFunctions.clamp(outputSpeed, -1, 1), MathFunctions.clamp(outputHeading, -1, 1));
    }

    private void setP2PMotorPowers(double scaleFactor) {
        double xPower = longitudinalController.calculate(currentPose.getX(), goalPose.getX());
        double yPower = lateralController.calculate(currentPose.getY(), goalPose.getY());
        double headingPower = headingController.calculate(MathFunctions.angleWrap(currentPose.getHeading()), MathFunctions.angleWrap(goalPose.getHeading()));

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

        if (isFollowingPath) {
            if ((MathFunctions.getDistance(currentPose, currentPath.getPose(currentPathIndex)) < distanceConstraint) && (currentPathIndex == currentPath.getSize() - 1)) {
                goalPose = currentPath.getPose(currentPath.getSize() - 1);
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
                double angleDiff = MathFunctions.angleWrap(angleToGoal - currentPose.getHeading());
                // so i think the angle thing is fine?

                setFollowerMotorPowers(dist, angleDiff);
            }
            return;
        }
        if (isP2Ping) {
            if (MathFunctions.getDistance(currentPose, goalPose) < holdPointRange && speed < speedConstraint && Math.abs(currentPose.getHeading() - goalPose.getHeading()) < headingConstraint) {
                isP2Ping = false;
                isFollowingPath = false;
                isHoldingPoint = true;
                return;
            }
            setP2PMotorPowers(1.0);
            return;
        }
        if (isHoldingPoint) {
            setP2PMotorPowers(holdPointScaleFactor);
        }
    }

    public boolean isFinished() {
        return !isP2Ping && !isFollowingPath;
    }

    // we keep following paths i guess. Once the last point in a "path" is reached, we will pid to point
}
