package org.firstinspires.ftc.teamcode.util.purepursuit;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FollowerConstants {
    public static double HOLD_POINT_SCALE_FACTOR = 0.3;
    public static double PATH_END_SPEED_CONSTRAINT = 1;
    public static double PATH_END_HEADING_CONSTRAINT = Math.toRadians(3);
    public static double PATH_END_DISTANCE_CONSTRAINT = 5;
    public static double HOLD_POINT_DISTANCE = 1;
    public static double LOOK_AHEAD_MIN_DISTANCE = 4;
    public static double LOOK_AHEAD_MAX_DISTANCE = 8;
    // TODO: these arent being used except KP LOOK AHEAD
    public static double KP_SPEED = 0.01;
    public static double KP_HEADING = 0.5;
    public static double KP_LOOK_AHEAD = 0.1;
    public static double KV_SPEED = 0.015;
    // these values will need to be tuned
    public static double MAX_VELOCITY = 60; // 50 in/s
    public static double MAX_ACCELERATION = 150; // in/s^2, same as deceleration
    public static PIDFCoefficients LONGITUDINAL_COEFFICIENTS = new PIDFCoefficients(0.01, 0, 0.0001, 0);
    public static PIDFCoefficients LATERAL_COEFFICIENTS = new PIDFCoefficients(0.01, 0, 0.0001, 0);
    public static PIDFCoefficients HEADING_COEFFICIENTS = new PIDFCoefficients(1, 0, 0.02, 0);

    public static String leftFrontMotorName = "front_left_drive";
    public static String rightFrontMotorName = "front_right_drive";
    public static String leftRearMotorName = "back_left_drive";
    public static String rightRearMotorName = "back_right_drive";
    public static DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    // future
    // private double voltage;
    // private double voltageCachingRate;

}
