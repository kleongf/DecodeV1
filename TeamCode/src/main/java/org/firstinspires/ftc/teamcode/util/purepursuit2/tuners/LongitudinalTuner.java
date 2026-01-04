package org.firstinspires.ftc.teamcode.util.purepursuit2.tuners;

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
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.util.purepursuit2.Matrix;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
@Autonomous(name = "Longitudinal tuner")
public class LongitudinalTuner extends OpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;
    private boolean stopped = false;
    private PoseUpdater poseUpdater;


    private Telemetry telemetryA;

    private double power = 0.0;
    private ElapsedTime elapsedTime;
    private ArrayList<double[]> dataPoints = new ArrayList<>();
    private ArrayList<double[]> powers = new ArrayList<>();
    private double kV = 0;
    private double kS = 0;
    private double kA = 0;

    /**
     * This initializes the drive motors as well as the cache of velocities and the FTC Dashboard
     * telemetry.
     */
    @Override
    public void init() {
        elapsedTime = new ElapsedTime();
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
        elapsedTime.reset();
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
        double dt = elapsedTime.seconds();
        if (!stopped) {
            if (gamepad1.cross || gamepad1.a) {
                stopped = true;
                for (DcMotorEx motor : motors) {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor.setPower(0);
                }
                double[][] dataPointsArray = dataPoints.toArray(new double[dataPoints.size()][]);
                double[][] powersArray = dataPoints.toArray(new double[dataPoints.size()][]);

                Matrix A = new Matrix(dataPointsArray);
                Matrix b = new Matrix(powersArray);

                // x = (AtA)^-1 Atb
                Matrix At = A.transpose();
                Matrix AtAInverse = A.multiply(At).inverse();
                Matrix AtB = At.multiply(b);
                Matrix x = AtAInverse.multiply(AtB);

                kS = x.get(0, 0);
                kV = x.get(1, 0);
                kA = x.get(2, 0);
            }
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightFront.setPower(power);
            rightRear.setPower(power);

            double v = poseUpdater.getVelocity().getYComponent();
            double a = poseUpdater.getAcceleration().getYComponent();

            dataPoints.add(new double[] {1.0, v, a});
            powers.add(new double[] {power});

            System.out.println("Power: " + power + ", Acceleration: " + a + ", Velocity: " + v);

            power += 0.005;
        } else {
            telemetryA.addData("kV", kV);
            telemetryA.addData("kS", kS);
            telemetryA.addData("kA", kA);
        }

        elapsedTime.reset();
        poseUpdater.update();
        telemetryA.update();
    }
}
