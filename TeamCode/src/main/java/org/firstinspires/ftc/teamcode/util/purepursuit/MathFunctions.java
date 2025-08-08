package org.firstinspires.ftc.teamcode.util.purepursuit;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

import java.util.Comparator;

public class MathFunctions {
    public static double getDistance(Point2D a, Point2D b) {
        return Math.sqrt(Math.pow((a.getX()-b.getX()), 2) + Math.pow((a.getY()-b.getY()), 2));
    }

    public static Pose2D getGoalPose(double lookAheadDistance, Pose2D currentPos, Path2D path, int i) {
        double x1 = path.getPose(i).getX() - currentPos.getX();
        double y1 = path.getPose(i).getY() - currentPos.getY();
        double x2 = path.getPose(i+1).getX() - currentPos.getX();
        double y2 = path.getPose(i+1).getY() - currentPos.getY();

        double dx = x2 - x1;
        double dy = y2 - y1;

        double dr = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
        double det = x1 * y2 - x2 * y1;

        double discriminant = Math.pow(lookAheadDistance, 2) * Math.pow(dr, 2) - Math.pow(det, 2);

        if (discriminant >= 0) {
            Pose2D sol1 = new Pose2D(
                    (det * dy + Math.signum(dy) * dx * Math.sqrt(discriminant)) / Math.pow(dr, 2),
                    (-det * dx + Math.abs(dy) * Math.sqrt(discriminant)) / Math.pow(dr, 2),
                    0
            );
            // use tangent
            sol1.setHeading(Math.atan2(sol1.getY() - currentPos.getY(), sol1.getX() - currentPos.getX()));

            Pose2D sol2 = new Pose2D(
                    (det * dy - Math.signum(dy) * dx * Math.sqrt(discriminant)) / Math.pow(dr, 2),
                    (-det * dx + Math.abs(dy) * Math.sqrt(discriminant)) / Math.pow(dr, 2),
                    0
            );
            // use tangent
            sol2.setHeading(Math.atan2(sol2.getY() - currentPos.getY(), sol2.getX() - currentPos.getX()));

            double minX = Math.min(path.getPose(i).getX(), path.getPose(i + 1).getX());
            double maxX = Math.max(path.getPose(i).getX(), path.getPose(i + 1).getX());
            double minY = Math.min(path.getPose(i).getY(), path.getPose(i + 1).getY());
            double maxY = Math.max(path.getPose(i).getY(), path.getPose(i + 1).getY());

            // check if both solutions are in range
            if (((sol1.getX() >= minX && sol1.getX() <= maxX) && (sol1.getY() >= minY && sol1.getY() <= maxY)) && ((sol2.getX() >= minX && sol2.getX() <= maxX) && (sol2.getY() >= minY && sol2.getY() <= maxY))) {
                if (getDistance(path.getPose(i+1), sol1) < getDistance(path.getPose(i+1), sol2)) {
                    return sol1;
                } else {
                    return sol2;
                }
            }
            // check if one solution in range
            if ((sol1.getX() >= minX && sol1.getX() <= maxX) && (sol1.getY() >= minY && sol1.getY() <= maxY)) {
                return sol1;
            } else {
                return sol2;
            }
        }
        // if theres no solution or both not in range then return the last pose
        Pose2D goalPose = new Pose2D(path.getPose(i+1).getX(), path.getPose(i+1).getY(), 0);
        goalPose.setHeading(Math.atan2(goalPose.getY() - currentPos.getY(), goalPose.getX() - currentPos.getX()));
        return goalPose;
    }

    public static double angleWrap(double radians) {
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

    public static Pose addPoses(Pose one, Pose two) {
        return new Pose(one.getX() + two.getX(), one.getY() + two.getY(), one.getHeading() + two.getHeading());
    }

    public static Pose subtractPoses(Pose one, Pose two) {
        return new Pose(one.getX() - two.getX(), one.getY() - two.getY(), one.getHeading() - two.getHeading());
    }

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        return angle < (double)0.0F ? angle + (Math.PI * 2D) : angle;
    }

    public static Pose rotatePose(Pose pose, double theta, boolean rotateHeading) {
        double x = pose.getX() * Math.cos(theta) - pose.getY() * Math.sin(theta);
        double y = pose.getX() * Math.sin(theta) + pose.getY() * Math.cos(theta);
        double heading = rotateHeading ? normalizeAngle(pose.getHeading() + theta) : pose.getHeading();
        return new Pose(x, y, heading);
    }

}
