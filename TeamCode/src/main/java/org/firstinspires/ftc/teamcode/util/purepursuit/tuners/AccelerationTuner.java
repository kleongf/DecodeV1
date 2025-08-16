package org.firstinspires.ftc.teamcode.util.purepursuit.tuners;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
@Autonomous(name = "Acceleration tuner")
public class AccelerationTuner extends OpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    private PoseUpdater poseUpdater;


    private Telemetry telemetryA;

    private double power = 0.0;

    /**
     * This initializes the drive motors as well as the cache of velocities and the FTC Dashboard
     * telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        poseUpdater = new PoseUpdater(hardwareMap, FConstants.class, LConstants.class);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("The robot will increase power over time by 0.005 per loop.");
        telemetryA.addLine("Make sure you have enough room!");
        telemetryA.addLine("Press CROSS or A on game pad 1 to stop.");
        telemetryA.addData("pose", poseUpdater.getPose());
        telemetryA.update();
    }

    /**
     * This starts the OpMode by setting the drive motors to run forward at full power.
     */
    @Override
    public void start() {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing CROSS or A on
     * game pad 1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run forward enough, these last velocities recorded are
     * averaged and printed.
     */
    @Override
    public void loop() {
        if (gamepad1.cross || gamepad1.a) {
            for (DcMotorEx motor : motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setPower(0);
            }
            requestOpModeStop();
        }
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);

        double acceleration = poseUpdater.getAcceleration().getXComponent();

        System.out.println("Power: " + power + ", Acceleration: " + acceleration);

        power += 0.005;

        poseUpdater.update();

        // goal is to graph power (y-axis) over acceleration (x-axis). this will give us a scaling factor for our vector.
    }
}
