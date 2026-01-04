package org.firstinspires.ftc.teamcode.util.purepursuit2;

import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.FRICTION_CONSTANT;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.HEADING_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.HOLD_POINT_SCALE_FACTOR;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.LATERAL_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.LONGITUDINAL_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.LOOK_AHEAD_DISTANCE;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.MAX_ACCELERATION;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.NOMINAL_VOLTAGE;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.PATH_END_DISTANCE_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.VOLTAGE_COMP_AUTO;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;

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
    public PIDFController longitudinalController;
    public PIDFController lateralController;
    public PIDFController headingController;
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
    private double pathEndDistanceConstraint;
    private double pathEndHeadingConstraint;
    private double pathEndSpeedConstraint;

    public PPFollower(HardwareMap hardwareMap) {
        this.localizer = new PPLocalizer(hardwareMap);
        this.currentPose = new Pose(0, 0, 0);
        this.longitudinalController = new PIDFController(LONGITUDINAL_COEFFICIENTS.kp, LONGITUDINAL_COEFFICIENTS.ki, LONGITUDINAL_COEFFICIENTS.kd, LONGITUDINAL_COEFFICIENTS.kf);
        this.lateralController = new PIDFController(LATERAL_COEFFICIENTS.kp, LATERAL_COEFFICIENTS.ki, LATERAL_COEFFICIENTS.kd, LATERAL_COEFFICIENTS.kf);
        this.headingController = new PIDFController(HEADING_COEFFICIENTS.kp, HEADING_COEFFICIENTS.ki, HEADING_COEFFICIENTS.kd, HEADING_COEFFICIENTS.kf);
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
    private void calculateGoalPose() {
        double posX = currentPose.getX();
        double posY = currentPose.getY();
        Pose goal = (lastFoundIndex == currentPath.getSize()-1) ? currentPath.getPose(lastFoundIndex) : currentPath.getPose(lastFoundIndex+1);

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

                Pose sol1 = new Pose(sol_x1 + posX, sol_y1 + posY, 0);
                Pose sol2 = new Pose(sol_x2 + posX, sol_y2 + posY, 0);
                // if it's not tangent we try to go to its normal heading
                if (!currentPath.isTangent()) {
                    sol1.setHeading(currentPath.getPose(i).getHeading());
                    sol2.setHeading(currentPath.getPose(i).getHeading());
                } else {
                    sol1.setHeading(Math.atan2(sol1.getY()-currentPose.getY(), sol1.getX()-currentPose.getX()));
                    sol2.setHeading(Math.atan2(sol2.getY()-currentPose.getY(), sol2.getX()-currentPose.getX()));
                }

                double minX = Math.min(currentPath.getPose(i).getX(), currentPath.getPose(i + 1).getX());
                double maxX = Math.max(currentPath.getPose(i).getX(), currentPath.getPose(i + 1).getX());
                double minY = Math.min(currentPath.getPose(i).getY(), currentPath.getPose(i + 1).getY());
                double maxY = Math.max(currentPath.getPose(i).getY(), currentPath.getPose(i + 1).getY());

                if (((sol1.getX() >= minX && sol1.getX() <= maxX) && (sol1.getY() >= minY && sol1.getY() <= maxY)) || ((sol2.getX() >= minX && sol2.getX() <= maxX) && (sol2.getY() >= minY && sol2.getY() <= maxY))) {
                    if (((sol1.getX() >= minX && sol1.getX() <= maxX) && (sol1.getY() >= minY && sol1.getY() <= maxY)) && ((sol2.getX() >= minX && sol2.getX() <= maxX) && (sol2.getY() >= minY && sol2.getY() <= maxY))) {
                        if (MathUtil.distance(currentPath.getPose(i + 1), sol1) < MathUtil.distance(currentPath.getPose(i + 1), sol2)) {
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

                    if (MathUtil.distance(goal, currentPath.getPose(i + 1)) < MathUtil.distance(currentPose, currentPath.getPose(i+1))) {
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

    // From WPILib (and some paper): For a DC Motor, V = kS * sgn(x) + kV * x' + kA * x''
    // kS: oppose friction
    // kV: target velocity
    // kA: target acceleration

    // moves in the general direction of the goal pose.
    public void moveToPose(Pose pose) {
        double dy = pose.getY()-currentPose.getY();
        double dx = pose.getX()-currentPose.getX();
        double dTheta = MathUtil.normalizeAngle(pose.getHeading()-currentPose.getHeading());

        // TODO: New Formulas for this: Ks*signum() + KV * desiredV + Ka * desired A
        // if the desired velocity has been reached, remove kA
        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());

        // coordinate transform matrix
        Matrix C = new Matrix(new double[][]{
                {sinH, -cosH, 0},
                {cosH,  sinH, 0},
                {0,     0,    1},
        });

        Matrix X = new Matrix(new double[][]{
                {dx, dy, dTheta}
        }).transpose().normalize(); // needed to transpose this first and need to normalize

        Matrix B = C.multiply(X);

        // kS matrix
        Matrix sgn = new Matrix(new double[][]{
                {KS_X * Math.signum(B.get(0, 0)), KS_Y * Math.signum(B.get(1, 0)), 0},
        }).transpose();

        // kV matrix transformation
        Matrix V = new Matrix(new double[][]{
                {KV_X * maxVelocity, 0, 0},
                {0,  KV_Y * maxVelocity, 0},
                {0,     0,    KV_HEADING},
        });

        // kA matrix transformation
        Matrix A = new Matrix(new double[][]{
                {KA_X * maxAcceleration, 0, 0},
                {0,  KA_Y * maxAcceleration, 0},
                {0,     0,    0},
        });

        Matrix T = localizer.getSpeed() >= maxVelocity ? sgn.add(V.multiply(B)) : sgn.add(V.multiply(B)).add(A.multiply(B));


        double xPower = T.get(0, 0);
        double yPower = T.get(1, 0);
        double thetaPower = T.get(1, 0);

        double total = Math.abs(xPower) + Math.abs(yPower) + Math.abs(thetaPower);
        xPower /= total;
        yPower /= total;
        thetaPower /= total;

        // thetaPower is negative because our coordinate system. also no negative scalefactor here, it dont make sense, it would reverse error
        setMotorPowers(xPower,  yPower, -thetaPower);
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
        // TODO: new idea: decelerate to a certain velocity, then remove the kA feedforward, no need for tuning quadratic damping
        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());

        double outX = lateralController.calculate(currentPose.getX(), goalPose.getX());
        double outY = longitudinalController.calculate(currentPose.getY(), goalPose.getY());
        double outHeading = -headingController.calculate(MathUtil.normalizeAngle(currentPose.getHeading()), MathUtil.normalizeAngle(goalPose.getHeading()));

        Matrix C = new Matrix(new double[][]{
                {sinH, -cosH, 0},
                {cosH,  sinH, 0},
                {0,     0,    1},
        });

        Matrix X = new Matrix(new double[][]{
                {outX, outY, outHeading}
        }).transpose();

        Matrix B = C.multiply(X);

        double xPower =  B.get(0, 0);
        double yPower =  B.get(1, 0);
        double headingPower = B.get(2, 0);

        double total = Math.abs(xPower) + Math.abs(yPower) + Math.abs(headingPower);

        if (total > 1) {
            xPower /= total;
            yPower /= total;
            headingPower /= total;
        }

        Matrix V = new Matrix(new double[][]{
                {localizer.getVelocity().getX(), localizer.getVelocity().getY(), 0}
        }).transpose();

        Matrix V2 = C.multiply(V);

        double xVel = V2.get(0, 0);;
        double yVel = V2.get(1, 0);;

        // TODO: we need separate constants for x and y. cannot be the same, also name them lateral and longitudinal
        if (xVel > PID_MAX_VELOCITY) {
            xPower += KA_X * -MAX_ACCELERATION * Math.signum(xVel); // brake, so that we go in opposite direction
        }

        if (yVel > PID_MAX_VELOCITY) {
            yPower += KA_Y * -MAX_ACCELERATION * Math.signum(yVel); // brake, so that we go in opposite direction
        }

        setMotorPowers(scaleFactor * xPower, scaleFactor * yPower, scaleFactor * headingPower);
    }

    public void followPath(PPPath path) {
        state = PPState.FOLLOWING_PATH;
        lookAheadDistance = path.getLookAheadDistance();
        maxVelocity = path.getMaxVelocity();
        maxAcceleration = path.getMaxAcceleration();
        pathEndSpeedConstraint = path.getPathEndSpeedConstraint();
        pathEndHeadingConstraint = path.getPathEndHeadingConstraint();
        pathEndDistanceConstraint = path.getPathEndDistanceConstraint();
        currentPath = path;
        currentPathIndex = 0;
        lastFoundIndex = 0;
    }

    public void update() {

        localizer.update();
        currentPose = localizer.getPose();
        double speed = localizer.getSpeed();
        double distanceToEnd = (speed * speed) / (2 * MAX_ACCELERATION);

        switch (state) {
            case IDLE:
                break;
            case FOLLOWING_PATH:
                // the second condition is a better catch, so that we don't go backwards from pure pursuit
                if ((MathUtil.distance(currentPose, currentPath.getPose(currentPath.getSize()-1)) < distanceToEnd) || MathUtil.distance(currentPose, currentPath.getPose(currentPath.getSize()-1)) < lookAheadDistance) {
                    goalPose = currentPath.getPose(currentPath.getSize() - 1);
                    state = PPState.PID_TO_POINT;
                } else {
                    calculateGoalPose();
                    if (currentPath.isReversed() && currentPath.isTangent()) {
                        moveToPose(MathUtil.reverseHeading(goalPose));
                    } else {
                        moveToPose(goalPose);
                    }
                }
                break;
            case PID_TO_POINT:
                if (MathUtil.distance(currentPose, goalPose) < pathEndDistanceConstraint && speed < pathEndSpeedConstraint && Math.abs(MathUtil.normalizeAngle(currentPose.getHeading()-goalPose.getHeading())) < pathEndHeadingConstraint) {
                    state = PPState.HOLDING_POINT;
                } else {
                    PIDToPose(1);
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
                currentPath = null;
                currentPathIndex = 0;
                lastFoundIndex = 0;
                PIDToPose(HOLD_POINT_SCALE_FACTOR);
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
        currentPath = null;
        currentPathIndex = 0;
        lastFoundIndex = 0;
    }

    public boolean isBusy() {
        return (state == PPState.FOLLOWING_PATH || state == PPState.PID_TO_POINT);
    }
}
