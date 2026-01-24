package org.firstinspires.ftc.teamcode.util.hardware;

import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.KQ_X;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.KQ_Y;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.X_ZPA;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.Y_ZPA;

import android.util.Log;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;
import org.firstinspires.ftc.teamcode.util.purepursuit.MathFunctions;
import org.firstinspires.ftc.teamcode.util.purepursuit2.MathUtil;
import org.firstinspires.ftc.teamcode.util.purepursuit2.Matrix;

public class SpecializedDrivetrain {
    private enum DrivetrainState {
        TELEOP_DRIVE,
        FOLLOWING_PATH,
        PID_TO_POSE
    }
    private ElapsedTime kickTimer;
    private double MAX_ACCELERATION = 150;
    private double MIN_DISTANCE_TO_END = 10;
    private double END_DISTANCE_CONSTRAINT = 4;
    private double END_VELOCITY_CONSTRAINT = 4;
    private double END_HEADING_CONSTRAINT = Math.toRadians(4);
    private Pose goalPose;
    private SimplePathChain currentPath;
    private int currentPathIndex;
    private Pose currentPose;
    private DrivetrainState state = DrivetrainState.TELEOP_DRIVE;
    private DcMotorEx fl, bl, fr, br;
    private HardwareMap hardwareMap;
    // public VoltageCompFollower follower;
    public VoltageCompFollower follower;
    private double targetHeading = 0;
    private double kp = 0.4;
    private double kd = 0.015;
    private double lastError = 0;
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
    private double KICK_TIME = 0.7;
    private boolean IS_KICKING = false;

    public SpecializedDrivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        fl = this.hardwareMap.get(DcMotorEx.class, "front_left_drive");
        bl = this.hardwareMap.get(DcMotorEx.class, "back_left_drive");
        fr = this.hardwareMap.get(DcMotorEx.class, "front_right_drive");
        br = this.hardwareMap.get(DcMotorEx.class, "back_right_drive");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        kickTimer = new ElapsedTime();
    }

    public void setFieldCentricMovementVectors(double forward, double strafe, double heading) {
        double botHeading = follower.getPose().getHeading();
        double x = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double y = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        double rx = heading;

        setMotorPowers(x, y, rx);

        targetHeading = currentPose.getHeading();
    }

    private void setMotorPowers(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);

        Log.d("fl power", String.valueOf(frontLeftPower));
        Log.d("bl power", String.valueOf(backLeftPower));
        Log.d("fr power", String.valueOf(frontRightPower));
        Log.d("br power", String.valueOf(backRightPower));
    }

    private double getDistance(Pose a, Pose b) {
        return Math.hypot(a.getX()-b.getX(), a.getY()-b.getY());
    }

    public void setHeadingLockFieldCentricMovementVectors(double forward, double strafe, double heading) {
        double botHeading = follower.getPose().getHeading();
        double x = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double y = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        // this is inverted so powers are inverted. no big deal i guess
        double error = MathFunctions.angleWrap(follower.getPose().getHeading()-targetHeading);
        double rx = kp * (error) + kd * (error-lastError) / period;

        setMotorPowers(x, y, rx);
        lastError = error;
    }

    public void setStartingPose(Pose p) {
        follower.setStartingPose(p);
        targetHeading = p.getHeading();
        currentPose = p;
    }

    private void pidToPose() {
        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());

        double errorHeading = MathFunctions.angleWrap(goalPose.getHeading()-follower.getPose().getHeading());
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

        double total = Math.abs(xPower) + Math.abs(yPower) + Math.abs(headingPower);

        if (total > 1) {
            xPower /= total;
            yPower /= total;
            headingPower /= total;
        }

        lastXError = robotErrX;
        lastYError = robotErrY;
        lastHeadingError = robotErrHeading;

        setMotorPowers(xPower, yPower, -headingPower);
    }

    private void moveToPose(Pose pose, double scaleFactor) {
        double dy = pose.getY()-currentPose.getY();
        double dx = pose.getX()-currentPose.getX();
        double dTheta = MathFunctions.angleWrap(pose.getHeading()-currentPose.getHeading());

        // concern: kX and kY are not being prioritized
        // solution: ensure that they make up at least 50% of the power.
        // kinda problem now is that power decreases with distance, but heading doesn't
        // afterwards u should still normalize so that u get 1 and proportionally more x and y movement
        // the max kTheta occurs when you are 180 degrees off, so pi * kTheta = 0.5, kTheta = 0.5/pi

        double kY = 1; // doesnt matter but this is a better number ig
        double kX = 1;
        double kTheta = 0.5/Math.PI;

        double outX = kX * dx;
        double outY = kY * dy;
        double outHeading = kTheta * dTheta;

        // 4) Convert GLOBAL outputs to ROBOT-LOCAL frame using current heading:
        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());

        double xPower =  sinH * outX  -  cosH * outY;
        double yPower =  cosH * outX  +  sinH * outY;
        double thetaPower = outHeading; // rotation is already body-centric sign

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

        setMotorPowers(scaleFactor * xPower, scaleFactor * yPower, Math.abs(scaleFactor) * -thetaPower);
    }

    public void breakFollowing() {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        state = DrivetrainState.TELEOP_DRIVE;
        currentPathIndex = 0;
        currentPath = null;
        goalPose = null;
        IS_KICKING = false;
        targetHeading = currentPose.getHeading();
    }

    public void followPath(SimplePathChain path) {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        state = DrivetrainState.FOLLOWING_PATH;
        IS_KICKING = false;
        currentPathIndex = 0;
        currentPath = path;
        goalPose = path.getPath(0).getEndPose();
    }

    public void kick(SimplePathChain path) {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        kickTimer.reset();
        IS_KICKING = true;
        state = DrivetrainState.FOLLOWING_PATH;
        currentPathIndex = 0;
        currentPath = path;
        goalPose = path.getPath(0).getEndPose();
    }

    public void update(double gpx, double gpy, double gprx) {
        follower.update();
        currentPose = follower.getPose();

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        switch (state) {
            case TELEOP_DRIVE:
                if (Math.abs(gprx) > 0) {
                    setFieldCentricMovementVectors(gpx, gpy, gprx);
                } else {
                    setHeadingLockFieldCentricMovementVectors(gpx, gpy, gprx);
                }
                break;
            case FOLLOWING_PATH:
                double distanceToEnd = Math.pow(follower.getVelocity().getMagnitude(), 2) / (2 * MAX_ACCELERATION);
                if (currentPath == null) {return;}
                if (goalPose == null) {return;}
                if (IS_KICKING && kickTimer.seconds() > KICK_TIME) {
                    breakFollowing();
                    return;
                }

                if (getDistance(currentPose, goalPose) < MIN_DISTANCE_TO_END || getDistance(currentPose, goalPose) < distanceToEnd) {
                    state = DrivetrainState.PID_TO_POSE;
                }
                moveToPose(goalPose, 1);
                break;
            case PID_TO_POSE:
                if (goalPose == null) {return;}
                if (IS_KICKING && kickTimer.seconds() > KICK_TIME) {
                    breakFollowing();
                    return;
                }
                boolean isLastPath = currentPathIndex == currentPath.getSize()-1;

                if (getDistance(currentPose, goalPose) < END_DISTANCE_CONSTRAINT && follower.getVelocity().getMagnitude() < END_VELOCITY_CONSTRAINT && Math.abs(MathFunctions.angleWrap(currentPose.getHeading()-goalPose.getHeading())) < END_HEADING_CONSTRAINT && isLastPath) {
                    breakFollowing();
                } else if (getDistance(currentPose, goalPose) < END_DISTANCE_CONSTRAINT * 4 && follower.getVelocity().getMagnitude() < END_VELOCITY_CONSTRAINT * 8 && Math.abs(MathFunctions.angleWrap(currentPose.getHeading()-goalPose.getHeading())) < END_HEADING_CONSTRAINT * 4 && !isLastPath) {
                    // more generous constraints with waypoints
                    currentPathIndex++;
                    goalPose = currentPath.getPath(currentPathIndex).getEndPose();
                    state = DrivetrainState.FOLLOWING_PATH;
                } else {
                    pidToPose();
                }
                break;
        }
    }

    public Pose getGoalPose() {return goalPose;}
    public String getState() {
        if (state == DrivetrainState.FOLLOWING_PATH) {
            return "FOLLOWING PATH";
        } else if (state == DrivetrainState.PID_TO_POSE) {
            return "PID TO POSE";
        }
        return "TELEOP DRIVE";
    }
    public void start() {
        breakFollowing();
    }
}
