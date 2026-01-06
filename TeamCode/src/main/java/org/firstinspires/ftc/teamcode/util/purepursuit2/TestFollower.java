package org.firstinspires.ftc.teamcode.util.purepursuit2;

import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.LATERAL_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.LONGITUDINAL_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.leftFrontMotorDirection;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.leftRearMotorDirection;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.rightFrontMotorDirection;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.rightRearMotorDirection;
import static org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants.rightRearMotorName;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;

public class TestFollower {
    private PPLocalizer localizer;
    public Pose currentPose;
    private Pose goalPose;
    public PIDFController longitudinalController;
    public PIDFController lateralController;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearLeft;
    private DcMotorEx rearRight;

    public TestFollower(HardwareMap hardwareMap) {
        this.localizer = new PPLocalizer(hardwareMap);
        this.currentPose = new Pose(0, 0, 0);
        this.longitudinalController = new PIDFController(LONGITUDINAL_COEFFICIENTS.kp, LONGITUDINAL_COEFFICIENTS.ki, LONGITUDINAL_COEFFICIENTS.kd, LONGITUDINAL_COEFFICIENTS.kf);
        this.lateralController = new PIDFController(LATERAL_COEFFICIENTS.kp, LATERAL_COEFFICIENTS.ki, LATERAL_COEFFICIENTS.kd, LATERAL_COEFFICIENTS.kf);

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
        currentPose = startPose;
        localizer.setStartPose(startPose);
    }

    private void setMotorPowers(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(MathUtil.clamp(frontLeftPower, -1, 1));
        rearLeft.setPower(MathUtil.clamp(backLeftPower, -1, 1));
        frontRight.setPower(MathUtil.clamp(frontRightPower, -1, 1));
        rearRight.setPower(MathUtil.clamp(backRightPower, -1, 1));
    }

    private void PIDToPose(double scaleFactor) {
        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());

        double outX = lateralController.calculate(currentPose.getX(), goalPose.getX());
        double outY = longitudinalController.calculate(currentPose.getY(), goalPose.getY());
        // double outHeading = -headingController.calculate(MathUtil.normalizeAngle(currentPose.getHeading()), MathUtil.normalizeAngle(goalPose.getHeading()));

        double headingError = MathUtil.normalizeAngle(goalPose.getHeading() - currentPose.getHeading());
        double outHeading = 1.5 * headingError; // simplified
        // double outHeading = -headingController.calculate(0, headingError);

        // double outHeading = 1.5 * MathUtil.normalizeAngle(goalPose.getHeading()-currentPose.getHeading());
        System.out.println("OUT HEADING: " + outHeading);
        System.out.println("GOAL POSE HEADING: " + goalPose.getHeading());
        System.out.println("CURRENT POSE HEADING: " + currentPose.getHeading());

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
        System.out.println("XPOWER: " + xPower);
        System.out.println("YPOWER: " + yPower);
        System.out.println("HEADING POWER: " + headingPower);
        // scaleFactor * headingPower

        setMotorPowers(scaleFactor * xPower, scaleFactor * yPower, scaleFactor * -headingPower);
    }

    public void setGoalPose(Pose pose) {
        goalPose = pose;
    }


    public void update() {
        localizer.update();
        currentPose = localizer.getPose();

        PIDToPose(1);

    }
}
