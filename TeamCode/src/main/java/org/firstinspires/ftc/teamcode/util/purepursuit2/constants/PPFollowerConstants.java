package org.firstinspires.ftc.teamcode.util.purepursuit2.constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.util.purepursuit2.PIDFCoefficients;

public class PPFollowerConstants {
    public static boolean VOLTAGE_COMP_AUTO = true;
    public static double FRICTION_CONSTANT = 0.15;
    public static double NOMINAL_VOLTAGE = 12.7;
    public static double HOLD_POINT_SCALE_FACTOR = 0.3;
    public static double PATH_END_SPEED_CONSTRAINT = 2;
    public static double PATH_END_HEADING_CONSTRAINT = Math.toRadians(2);
    public static double PATH_END_DISTANCE_CONSTRAINT = 2;
    public static double LOOK_AHEAD_DISTANCE = 10;
    public static double MAX_VELOCITY = 60; // 60 in/s
    public static double MAX_ACCELERATION = 150; // in/s^2, same as deceleration
    public static double PID_MAX_VELOCITY = 15; // a random number but it's ok
    public static double KS_X; // need to be tuned
    public static double KS_Y;
    public static double KV_X;
    public static double KV_Y;
    public static double KA_X;
    public static double KA_Y;
    public static double KV_HEADING; // will be easy once we know the kV and kA and kS, also not as important. compensate accoringly to the previous 1-2-8
    public static PIDFCoefficients LONGITUDINAL_COEFFICIENTS = new PIDFCoefficients(0.02, 0, 0.0006, 0);
    public static PIDFCoefficients LATERAL_COEFFICIENTS = new PIDFCoefficients(0.05, 0, 0.0015, 0);
    public static PIDFCoefficients HEADING_COEFFICIENTS = new PIDFCoefficients(1.5, 0, 0.03, 0);

    public static String leftFrontMotorName = "front_left_drive";
    public static String rightFrontMotorName = "front_right_drive";
    public static String leftRearMotorName = "back_left_drive";
    public static String rightRearMotorName = "back_right_drive";
    public static DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

}

