package org.firstinspires.ftc.teamcode.util.purepursuit;

import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;
import static org.firstinspires.ftc.teamcode.util.purepursuit.PurePursuitConstants.*;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class PurePursuit {
    private PurePursuitState purePursuitState = PurePursuitState.IDLE;
    private Localizer localizer;
    public Pose2D currentPose;
    private Pose2D goalPose;
    public PIDFController longitudinalController;
    public PIDFController lateralController;
    public PIDFController headingController;

    private Path2D currentPath;
    private int currentPathIndex;
    private int lastFoundIndex;
    private double maxPower = 1.0;
    private double lookAheadDistance = LOOK_AHEAD_DISTANCE;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearLeft;
    private DcMotorEx rearRight;
    private VoltageSensor voltageSensor;
    private double vx = 0;
    private double vy = 0;
    private double vr = 0;

    public PurePursuit(HardwareMap hardwareMap) {
        this.localizer = new Localizer(hardwareMap);
        this.currentPose = new Pose2D(0, 0, 0);
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

        this.voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    public void setStartingPose(Pose startPose) {
        currentPose = new Pose2D(startPose.getX(), startPose.getY(), startPose.getHeading());
        localizer.setStartPose(startPose);
    }



    private double getVoltageScaler() {
        return (NOMINAL_VOLTAGE - (NOMINAL_VOLTAGE * FRICTION_CONSTANT)) / (voltageSensor.getVoltage() - ((Math.pow(NOMINAL_VOLTAGE, 2) / voltageSensor.getVoltage()) * FRICTION_CONSTANT));
    }

    private void calculateGoalPose() {
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

    public void moveToPose(Pose2D pose, double scaleFactor) {
        double dy = pose.getY()-currentPose.getY();
        double dx = pose.getX()-currentPose.getX();
        double dTheta = MathFunctions.angleWrap(pose.getHeading()-currentPose.getHeading());

        double kY = 1.0;
        double kX = 2.0;
        double kTheta = 8.0;

        double xPower = (Math.sin(currentPose.getHeading()) * dx - Math.cos(currentPose.getHeading()) * dy) * kX;
        double yPower = (Math.cos(currentPose.getHeading()) * dx + Math.sin(currentPose.getHeading()) * dy) * kY;
        double thetaPower = dTheta * kTheta;

        double total = Math.abs(xPower) + Math.abs(yPower) + Math.abs(thetaPower);
        xPower /= total;
        yPower /= total;
        thetaPower /= total;

        // thetaPower is negative because our coordinate system. also no negative scalefactor here, it dont make sense, it would reverse error
        setMotorPowers(scaleFactor * xPower, scaleFactor * yPower, Math.abs(scaleFactor) * -thetaPower);
    }

    private void setMotorPowers(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        if ((VOLTAGE_COMP_AUTO && purePursuitState != PurePursuitState.TELEOP_DRIVE) || (VOLTAGE_COMP_TELEOP && purePursuitState == PurePursuitState.TELEOP_DRIVE)) {
            double scaler = getVoltageScaler();
            frontLeftPower *= scaler;
            backLeftPower *= scaler;
            frontRightPower *= scaler;
            backRightPower *= scaler;
        }

        frontLeft.setPower(MathFunctions.clamp(frontLeftPower, -maxPower, maxPower));
        rearLeft.setPower(MathFunctions.clamp(backLeftPower, -maxPower, maxPower));
        frontRight.setPower(MathFunctions.clamp(frontRightPower, -maxPower, maxPower));
        rearRight.setPower(MathFunctions.clamp(backRightPower, -maxPower, maxPower));
    }

    private void PIDToPose(double scaleFactor) {
        double outX = lateralController.calculate(currentPose.getX(), goalPose.getX());
        double outY = longitudinalController.calculate(currentPose.getY(), goalPose.getY());
        double outHeading = -headingController.calculate(MathFunctions.angleWrap(currentPose.getHeading()), MathFunctions.angleWrap(goalPose.getHeading()));

        // 4) Convert GLOBAL outputs to ROBOT-LOCAL frame using current heading:
        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());

        double xPower =  sinH * outX  -  cosH * outY;
        double yPower =  cosH * outX  +  sinH * outY;
        double headingPower = outHeading; // rotation is already body-centric sign

        double total = Math.abs(xPower) + Math.abs(yPower) + Math.abs(headingPower);

        if (total > 1) {
            xPower /= total;
            yPower /= total;
            headingPower /= total;
        }

        setMotorPowers(scaleFactor * xPower, scaleFactor * yPower, scaleFactor * headingPower);
    }

    public void followPath(Path2D path) {
        purePursuitState = PurePursuitState.FOLLOWING_PATH;
        lookAheadDistance = path.getLookAheadDistance();
        maxPower = path.getMaxPower();
        currentPath = path;
        currentPathIndex = 0;
        lastFoundIndex = 0;
    }
    public void pidToPoint(Pose2D pose) {
        purePursuitState = PurePursuitState.PIDING_TO_POINT;
        goalPose = pose;
        currentPath = null;
        currentPathIndex = 0;
        lastFoundIndex = 0;
    }

    public void update() {
        localizer.update();
        currentPose = localizer.getPose2D();
        double speed = localizer.getSpeed();
        double distanceToEnd = (speed * speed) / (2 * MAX_ACCELERATION);

        switch (purePursuitState) {
            case IDLE:
                break;
            case TELEOP_DRIVE:
                setMotorPowers(vx, vy, vr);
                break;
            case FOLLOWING_PATH:
                if ((MathFunctions.getDistance(currentPose, currentPath.getPose(currentPath.getSize()-1)) < distanceToEnd) || MathFunctions.getDistance(currentPose, currentPath.getPose(currentPath.getSize()-1)) < PATH_END_DISTANCE_CONSTRAINT) {
                    goalPose = currentPath.getPose(currentPath.getSize() - 1);
                    purePursuitState = PurePursuitState.PIDING_TO_POINT;
                } else {
                    calculateGoalPose();
                    if (currentPath.isReversed()) {
                        moveToPose(MathFunctions.reverseHeading(goalPose), 1);
                    } else {
                        moveToPose(goalPose, 1);
                    }
                }
                break;
            case PIDING_TO_POINT:
                if (MathFunctions.getDistance(currentPose, goalPose) < PID_TO_POINT_END_DISTANCE_CONSTRAINT && speed < PID_TO_POINT_END_SPEED_CONSTRAINT && Math.abs(MathFunctions.angleWrap(currentPose.getHeading()-goalPose.getHeading())) < PID_TO_POINT_END_HEADING_CONSTRAINT) {
                    purePursuitState = PurePursuitState.HOLDING_POINT;
                } else {
                    calculateGoalPose();
                    PIDToPose(1.0);
                }
                break;
            case HOLDING_POINT:
                // resetting
                maxPower = 1.0;
                lookAheadDistance = LOOK_AHEAD_DISTANCE;
                PIDToPose(HOLD_POINT_SCALE_FACTOR);
                break;
        }
    }

    public void setTeleopMovement(double x, double y, double r) {
        vx = x;
        vy = y;
        vr = r;
    }

    public void startTeleopDrive() {
        purePursuitState = PurePursuitState.TELEOP_DRIVE;
        maxPower = 1.0;
        lookAheadDistance = LOOK_AHEAD_DISTANCE;
        currentPath = null;
        currentPathIndex = 0;
        lastFoundIndex = 0;

        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void breakFollowing() {
        purePursuitState = PurePursuitState.IDLE;
        maxPower = 1.0;
        lookAheadDistance = LOOK_AHEAD_DISTANCE;
        currentPath = null;
        currentPathIndex = 0;
        lastFoundIndex = 0;
    }

    public boolean isBusy() {
        return (purePursuitState == PurePursuitState.FOLLOWING_PATH || purePursuitState == PurePursuitState.PIDING_TO_POINT);
    }
}
