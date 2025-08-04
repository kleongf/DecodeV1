package org.firstinspires.ftc.teamcode.util.pathplanner;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class SafePathGenerator {
    private LazyTheta lazyTheta;
    private List<PathChain> pathChains; // rename this to pathChains?
    private List<TargetPose> poses;
    private VoltageCompFollower follower;
    private int index = 0;
    public SafePathGenerator(VoltageCompFollower follower, TargetPose...poses) {
        this.follower = follower;
        lazyTheta = new LazyTheta(8);
        pathChains = new ArrayList<>();
        this.poses = new ArrayList<>();
        this.poses.addAll(Arrays.asList(poses));
    }

    // call this method in init(), then you can use the paths in paths
    public void generatePaths() {
        TargetPose currentPose = poses.get(0);
        for (TargetPose nextPose: poses) {
            List<Node> bestPoints = lazyTheta.findPath(currentPose.getX(), currentPose.getY(), nextPose.getX(), nextPose.getY());
            List<Path> paths = new ArrayList<>();

            PathBuilder pathBuilder = new PathBuilder();
            for (int i = 0; i < bestPoints.size()-1; i++) {
                Path path = new Path(
                        new BezierLine(
                                new Point(bestPoints.get(i).x, bestPoints.get(i).y),
                                new Point(bestPoints.get(i+1).x, bestPoints.get(i+1).y)
                        )
                );
                if (i == bestPoints.size()-2) {
                    if (bestPoints.size() > 2) {
                        int lastDy = bestPoints.get(bestPoints.size()-2).y - bestPoints.get(bestPoints.size()-3).y;
                        int lastDx = bestPoints.get(bestPoints.size()-2).x - bestPoints.get(bestPoints.size()-3).x;
                        double angle = Math.atan2(lastDy, lastDx);
                        path.setLinearHeadingInterpolation(angle, nextPose.getHeading());
                    } else {
                        path.setLinearHeadingInterpolation(currentPose.getHeading(), nextPose.getHeading());
                    }
                } else {
                    path.setTangentHeadingInterpolation();
                }
                paths.add(path);
            }

            for (Path path: paths) {
                pathBuilder = pathBuilder.addPath(path);
            }
            PathChain pathChain = pathBuilder.build();
            pathChains.add(pathChain);
        }
    }
    // when following a path just call this
    public List<PathChain> getPaths() {
        return pathChains;
    }

    public void followNextPath(boolean holdEnd) {
        follower.followPath(pathChains.get(index), holdEnd);
        index++;
    }

    public void followPathAtIndex(int i, boolean holdEnd) {
        follower.followPath(pathChains.get(i), holdEnd);
    }

    public void setIndex(int idx) {
        index = idx;
    }

    public int getIndex() {
        return index;
    }
}
