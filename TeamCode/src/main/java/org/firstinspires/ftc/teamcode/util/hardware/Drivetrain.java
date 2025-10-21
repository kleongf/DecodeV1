package org.firstinspires.ftc.teamcode.util.hardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.util.purepursuit.MathFunctions;

public class Drivetrain {
    private DcMotorEx fl, bl, fr, br;
    private HardwareMap hardwareMap;
    public Follower follower;
    private PIDFController headingController;
    private double targetHeading = 0;
    private double prevHeading = 0;
    private double angularVelocity = 0;
    private ElapsedTime elapsedTime;

    public Drivetrain(HardwareMap hardwareMap) {
        this.headingController = new PIDFController(2,0,0.1,0); // strongar than pedro
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

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        // TODO: change this
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(270)));
        elapsedTime = new ElapsedTime();
    }

    public void setMovementVectors(double forward, double strafe, double heading) {
        double y = -forward; // Remember, Y stick value is reversed
        double x = strafe * 1.1; // Counteract imperfect strafing
        double rx = heading;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);
    }

    public void setFieldCentricMovementVectors(double forward, double strafe, double heading) {
        double botHeading = follower.getPose().getHeading();
        double x = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
//        x *= 1.1;
        double y = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        double rx = heading;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);

        targetHeading = follower.getPose().getHeading();
    }

    public void setHeadingLockFieldCentricMovementVectors(double forward, double strafe, double heading) {
        double botHeading = follower.getPose().getHeading();
        double x = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double y = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        double rx = -headingController.calculate(MathFunctions.angleWrap(follower.getPose().getHeading()));

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);
    }

    public void setStartingPose(Pose p) {
        follower.setStartingPose(p);
        targetHeading = p.getHeading();
    }

    public double getTotalAngularVelocity() {
        // x/y because weird coord system
        double maxVelocity = 160;
        return angularVelocity + (follower.getVelocity().getMagnitude() / maxVelocity) * Math.atan2(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent());
    }


    public void update() {
        angularVelocity = elapsedTime.seconds() > 0 ? (follower.getPose().getHeading() - prevHeading) / elapsedTime.seconds() : 0;
        prevHeading = follower.getPose().getHeading();
        elapsedTime.reset();
        follower.update();
    }
}
