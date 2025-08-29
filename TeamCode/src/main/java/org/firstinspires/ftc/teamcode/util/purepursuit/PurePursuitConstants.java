package org.firstinspires.ftc.teamcode.util.purepursuit;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class PurePursuitConstants {
    // path end constraints: what distance/speed we can go at before p2ping
    // p2p end constraint: what distance/speed we need to be at before holding point
    public static boolean VOLTAGE_COMP_AUTO = true;
    public static boolean VOLTAGE_COMP_TELEOP = true;
    public static double FRICTION_CONSTANT = 0.15;
    public static double NOMINAL_VOLTAGE = 12.7;

    public static double LOOK_AHEAD_DISTANCE = 10;
    public static double PATH_END_DISTANCE_CONSTRAINT = 5;
    public static double PID_TO_POINT_END_SPEED_CONSTRAINT = 1;
    public static double PID_TO_POINT_END_DISTANCE_CONSTRAINT = 1;
    public static double PID_TO_POINT_END_HEADING_CONSTRAINT = Math.toRadians(3);
    public static double HOLD_POINT_SCALE_FACTOR = 0.3;
    public static double MAX_ACCELERATION = 100; // in/s^2, same as deceleration
    // TODO: could probably replace the feedforward with frictionconstant
    public static double LONGITUDINAL_FEEDFORWARD = 0.1;
    public static double LATERAL_FEEDFORWARD = 0.1;
    public static PIDFCoefficients LONGITUDINAL_COEFFICIENTS = new PIDFCoefficients(0.12, 0, 0.012, 0);
    public static PIDFCoefficients LATERAL_COEFFICIENTS = new PIDFCoefficients(0.06, 0, 0.006, 0);

    public static PIDFCoefficients FAR_LONGITUDINAL_COEFFICIENTS = new PIDFCoefficients(0.1, 0, 0.005, 0);
    public static PIDFCoefficients FAR_LATERAL_COEFFICIENTS = new PIDFCoefficients(0.05, 0, 0.0001, 0);
    public static PIDFCoefficients HEADING_COEFFICIENTS = new PIDFCoefficients(1, 0, 0.04, 0);

    public static String leftFrontMotorName = "front_left_drive";
    public static String rightFrontMotorName = "front_right_drive";
    public static String leftRearMotorName = "back_left_drive";
    public static String rightRearMotorName = "back_right_drive";
    public static DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
}
