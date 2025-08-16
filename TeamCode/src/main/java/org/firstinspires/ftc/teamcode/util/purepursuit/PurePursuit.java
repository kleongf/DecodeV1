package org.firstinspires.ftc.teamcode.util.purepursuit;


import org.firstinspires.ftc.teamcode.util.controllers.HeadingPIDFController;
import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;
import static org.firstinspires.ftc.teamcode.util.purepursuit.PurePursuitConstants.*;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class PurePursuit {
    private Localizer localizer;
    public Pose2D currentPose;
    private Pose2D goalPose;
    private double speed;
    private boolean isFollowingPath = false;
    private boolean isP2Ping = false;
    private boolean isDecelerating = false;
    public PIDFController longitudinalController;
    public PIDFController lateralController;
    public PIDFController headingController;
    private double decelerationDistance;

    private Path2D currentPath;
    private int currentPathIndex;
    private int lastFoundIndex;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearLeft;
    private DcMotorEx rearRight;
    // TODO: um yeah so some changes we need to make:
    // we need to make power constant at 1, normalize vectors proportionally. these should be computed by distance and angle to target. we can subtract these to get power vectors and find how fast robot goes laterally and longitudinally, multiply them proportionally as well.
    // we could do the same thing with heading, just getting max heading speed and finding arc length or something
    // need to predict where robot will go based on its current position and heading, to the goal pos and heading. should make it better for braking.
    // use max acceleration? then use p controller?
    // i honestly think a mix is the best: once we hit the zero power acceleration, continue using the pure pursuit vector, but set translational+drive power to zero (only use heading) and not make it a unit vector
    // once velocity is lower like 10-20 m/s then pid to point

    // TODO: new implementation
    // when isDecelerating, use a velocity feedforward
    // formula is given as follows:
    // feedforward(x) sqrt(max(0, 2 * targetDecel * (x-coastDistance))
    // prob should do some math to do the same thing for y
    // this becomes our new vector, or the new scaling thing for our vector, idk

    // targetdecel is like 100 for example. thats what we want to decelerate at.
    // coastDistance = c^2/2(zpam) (c^2 is max velocity, or i guess here current velocity before decelerating)

    // the problem is executing this in two dimensions and with a heading vector
    // i think we just keep heading but scale the other two

    // feedforward = Kp * (targetVelocity - currentVelocity) + Kv * (targetVelocity)

    // you know what? i should make a function that maps motor power (0-1) to acceleration.
    // that way, when we decelerate, i just set the power to that value until speed decreases sufficiently.
    // finally, i can pid to point when speed is low enough (10 in/s)

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
    }

    public void setStartingPose(Pose startPose) {
        currentPose = new Pose2D(startPose.getX(), startPose.getY(), startPose.getHeading());
        localizer.setStartPose(startPose);
    }

    public double getPowerForAcceleration(double acc) {
        // TODO: find these values from the tuner, which will give us a least-squares regression line
        return 0.08 + 0.01 * Math.abs(acc);
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
            double discriminant = (LOOK_AHEAD_DISTANCE * LOOK_AHEAD_DISTANCE) * (dr * dr) - (det * det);

            if (discriminant >= 0) {
                double sol_x1 = (det * dy + Math.signum(dy) * dx * Math.sqrt(discriminant)) / (dr * dr);
                double sol_x2 = (det * dy - Math.signum(dy) * dx * Math.sqrt(discriminant)) / (dr * dr);
                double sol_y1 = (-det * dx + Math.abs(dy) * Math.sqrt(discriminant)) / (dr * dr);
                double sol_y2 = (-det * dx - Math.abs(dy) * Math.sqrt(discriminant)) / (dr * dr);

                Pose2D sol1 = new Pose2D(sol_x1 + posX, sol_y1 + posY, 0);
                Pose2D sol2 = new Pose2D(sol_x2 + posX, sol_y2 + posY, 0);
                sol1.setHeading(Math.atan2(sol1.getY()-currentPose.getY(), sol1.getX()-currentPose.getX()));
                sol2.setHeading(Math.atan2(sol2.getY()-currentPose.getY(), sol2.getX()-currentPose.getX()));

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

        // TODO: i need something to measure power set for x y and heading and determine how far it goes. Then I can get more accurate results.
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

    public double getRequiredAcceleration() {
        // a = vo^2/-2dx
//        double accelX = Math.pow(localizer.getVelocity().getX(), 2) / (-2 * (goalPose.getX() - currentPose.getX()));
//        double accelY = Math.pow(localizer.getVelocity().getY(), 2) / (-2 * (goalPose.getX() - currentPose.getY()));
//
//        double xRot = (Math.sin(currentPose.getHeading()) * accelX - Math.cos(currentPose.getHeading()) * accelY);
//        double yRot = (Math.cos(currentPose.getHeading()) * accelX + Math.sin(currentPose.getHeading()) * accelY);
//        return Math.sqrt(Math.pow(xRot, 2) + Math.pow(yRot, 2));
        // i could probably do this an easier way, let's just do distance lol
        return Math.abs((Math.pow(localizer.getSpeed(), 2)) / (-2 * MathFunctions.getDistance(currentPose, goalPose)));
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

    private void PIDToPose() {
        // i think this is also cooked and the power needs normalizing
        double outX = lateralController.calculate(currentPose.getX(), goalPose.getX());
        double outY = longitudinalController.calculate(currentPose.getY(), goalPose.getY());
        double outHeading = -headingController.calculate(MathFunctions.angleWrap(currentPose.getHeading()), MathFunctions.angleWrap(goalPose.getHeading()));

        // 4) Convert GLOBAL outputs to ROBOT-LOCAL frame using current heading:
        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());

        double xPower =  sinH * outX  -  cosH * outY;
        double yPower =  cosH * outX  +  sinH * outY;
        double headingPower = outHeading; // rotation is already body-centric sign

        System.out.println("X Power: " + xPower);
        System.out.println("Y Power: " + yPower);
        System.out.println("Heading Power: " + headingPower);
        System.out.println("Goal Heading: " + goalPose.getHeading());

        setMotorPowers(xPower, yPower, headingPower);
    }

    public void followPath(Path2D path) {
        currentPath = path;
        isP2Ping = false;
        isFollowingPath = true;
        isDecelerating = false;
        currentPathIndex = 0;
        lastFoundIndex = 0;
    }
    public void p2p(Pose2D pose) {
        goalPose = pose;
        isP2Ping = true;
        isFollowingPath = false;
        isDecelerating = false;
        currentPath = null;
        currentPathIndex = 0;
        lastFoundIndex = 0;
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
        }

        System.out.println("Distance TO End: " + distanceToEnd);
        if (isP2Ping) {
            System.out.println("P2PING");
        }

        if (isFollowingPath) {
            if (isDecelerating) {
                if (speed < PATH_END_SPEED_CONSTRAINT) {
                    isP2Ping = true;
                    goalPose = currentPath.getPose(currentPath.getSize() - 1);
                    isFollowingPath = false;
                    isDecelerating = false;
                    currentPath = null;
                    currentPathIndex = 0;
                    lastFoundIndex = 0;
                    return;
                }
                calculateGoalPose();
                // when distance = decelerationDistance multiplier = 1, when distance = 0 multiplier = 0
                // double multiplier = MathFunctions.clamp((MathFunctions.getDistance(currentPose, goalPose)/decelerationDistance), -1, 1);
                // now we find power for acceleration! so smart
                // wait hold up, wouldn't this just push the robot farther from goal point?
                // we need to do some math here
                // so we need to predict where robot will go based on its current position
                // make acceleration based on predicted position from goal point (made from
                // i like this idea actually
                double multiplier = -1 * getRequiredAcceleration();
                if (currentPath.isReversed()) {
                    Pose2D pose = new Pose2D(goalPose.getX(), goalPose.getY(), goalPose.getHeading()-Math.PI);
                    moveToPose(pose, 1 * multiplier);
                } else {
                    moveToPose(goalPose, 1 * multiplier);
                }
                return;
            } else if ((MathFunctions.getDistance(currentPose, currentPath.getPose(currentPath.getSize()-1)) < distanceToEnd)) {
                // this means we should start decelerating
                isDecelerating = true;
                decelerationDistance = distanceToEnd;
                calculateGoalPose();
                if (currentPath.isReversed()) {
                    Pose2D pose = new Pose2D(goalPose.getX(), goalPose.getY(), goalPose.getHeading()-Math.PI);
                    moveToPose(pose, 1);
                } else {
                    moveToPose(goalPose, 1);
                }
                return;
            } else {
                calculateGoalPose();
                if (currentPath.isReversed()) {
                    Pose2D pose = new Pose2D(goalPose.getX(), goalPose.getY(), goalPose.getHeading()-Math.PI);
                    moveToPose(pose, 1);
                } else {
                    moveToPose(goalPose, 1);
                }
                return;
            }
        }
        if (isP2Ping) {
            PIDToPose();
        }
    }

    public boolean isFinished() {
        return isP2Ping && !isFollowingPath && MathFunctions.getDistance(currentPose, goalPose) < END_DISTANCE_CONSTRAINT && Math.abs(MathFunctions.angleWrap(currentPose.getHeading()-goalPose.getHeading())) < END_HEADING_CONSTRAINT && speed < END_SPEED_CONSTRAINT;
    }
}
