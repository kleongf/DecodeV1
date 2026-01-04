package org.firstinspires.ftc.teamcode.util.purepursuit2;

import com.pedropathing.localization.Pose;

public class MathUtil {
    public static Pose reverseHeading(Pose pose) {
        return new Pose(pose.getX(), pose.getY(), pose.getHeading()-Math.PI);
    }

    public static double inchesToMeters(double inches) {
        return inches / 39.37;
    }

    public static double metersToInches(double meters) {
        return meters * 39.37;
    }

    public static double distance(Pose a, Pose b) {
        return Math.hypot(a.getX()-b.getX(), a.getY()-b.getY());
    }

    public static double normalizeAngle(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        // keep in mind that the result is in radians
        return radians;
    }

    public static double clamp(double num, double lower, double upper) {
        if (num < lower) {
            return lower;
        } else {
            return num > upper ? upper : num;
        }
    }
}
