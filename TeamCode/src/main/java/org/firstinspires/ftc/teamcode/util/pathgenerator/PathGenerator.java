package org.firstinspires.ftc.teamcode.util.pathgenerator;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class PathGenerator {
    private PathBuilder pathBuilder;
    private List<PathChain> paths;
    private List<ControlPoint> points;
    private VoltageCompFollower follower;
    private int index = 0;
    public PathGenerator(VoltageCompFollower follower, ControlPoint...points) {
        pathBuilder = new PathBuilder();
        paths = new ArrayList<>();
        this.points = new ArrayList<>();
        this.points.addAll(Arrays.asList(points));
    }

    // call this method in init(), then you can use the paths in paths
    public void generatePaths() {
        TargetPose currentPose = null;
        ArrayList<ControlPoint> controlPoints = new ArrayList<>();
        for (ControlPoint point: points) {
            if (point instanceof TargetPose) {
                if (currentPose == null) {
                    currentPose = (TargetPose) point;
                } else {
                    ArrayList<Point> allPoints = new ArrayList<>();
                    allPoints.add(new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN));
                    for (ControlPoint p: controlPoints) {
                        allPoints.add(new Point(p.getX(), p.getY(), Point.CARTESIAN));
                    }
                    allPoints.add(new Point(point.getX(), point.getY(), Point.CARTESIAN));
                    PathChain path = follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            allPoints
                                    )
                            )
                            .setLinearHeadingInterpolation(currentPose.getHeading(), ((TargetPose) point).getHeading())
                            .build();
                    paths.add(path);
                    currentPose = (TargetPose) point;
                    controlPoints.clear();
                }
            } else {
                controlPoints.add(point);
            }
        }
    }
    // when following a path just call this
    public List<PathChain> getPaths() {
        return paths;
    }

    public void followNextPath(boolean holdEnd) {
        follower.followPath(paths.get(index), holdEnd);
        index++;
    }

    public void followPathAtIndex(int i, boolean holdEnd) {
        follower.followPath(paths.get(i), holdEnd);
    }

    public void setIndex(int idx) {
        index = idx;
    }

    public int getIndex() {
        return index;
    }
}
