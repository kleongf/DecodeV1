package org.firstinspires.ftc.teamcode.util.purepursuit;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class PurePursuitConstants {
    public static double LOOK_AHEAD_DISTANCE = 10;
    public static double PATH_END_DISTANCE_CONSTRAINT = 5;
    public static double PATH_END_SPEED_CONSTRAINT = 20;
    public static double END_SPEED_CONSTRAINT = 1;
    public static double END_DISTANCE_CONSTRAINT = 1;
    public static double END_HEADING_CONSTRAINT = Math.toRadians(3);
    public static double MAX_ACCELERATION = 80; // in/s^2, same as deceleration on pedro zpam
    public static PIDFCoefficients LONGITUDINAL_COEFFICIENTS = new PIDFCoefficients(0.02, 0, 0.0002, 0);
    public static PIDFCoefficients LATERAL_COEFFICIENTS = new PIDFCoefficients(0.08, 0, 0.001, 0);
    public static PIDFCoefficients HEADING_COEFFICIENTS = new PIDFCoefficients(1.5, 0, 0.02, 0);

    public static String leftFrontMotorName = "front_left_drive";
    public static String rightFrontMotorName = "front_right_drive";
    public static String leftRearMotorName = "back_left_drive";
    public static String rightRearMotorName = "back_right_drive";
    public static DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
}
