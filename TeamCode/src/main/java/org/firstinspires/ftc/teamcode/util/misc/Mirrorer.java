package org.firstinspires.ftc.teamcode.util.misc;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.util.purepursuit.MathFunctions;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class Mirrorer {
    public static Pose mirror(Pose p) {
        return new Pose(144 - p.getX(), p.getY(), MathFunctions.angleWrap(Math.PI - p.getHeading()));
    }

    public static Point mirror(Point p) {
        return new Point(144 - p.getX(), p.getY(), Point.CARTESIAN);
    }

    public static double mirror(double x) {
        return MathFunctions.angleWrap(Math.PI - x);
    }
    public static PathChain mirror(PathChain p) {
        ArrayList<Path> newPaths = new ArrayList<>();
        for (int i = 0; i < p.size(); i++) {
            Path currentPath = p.getPath(i);
            ArrayList<Point> points = currentPath.getControlPoints();
            double startHeading = currentPath.getHeadingGoal(0);
            double endHeading = currentPath.getHeadingGoal(1);

            List<Point> newPoints = points.stream()
                    .map(Mirrorer::mirror) // Apply transformation
                    .collect(Collectors.toList());

            Path newPath = new Path(new BezierCurve((ArrayList<Point>) newPoints));
            newPath.setLinearHeadingInterpolation(mirror(startHeading), mirror(endHeading));
            newPaths.add(newPath);
        }
        return new PathChain(newPaths);
    }
}
