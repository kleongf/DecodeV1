package org.firstinspires.ftc.teamcode.util.purepursuit;

import java.util.ArrayList;
import java.util.Arrays;

public class Path2D {
    private final ArrayList<Pose2D> waypoints;
    private boolean reversed;
    private boolean tangent = true;

    public Path2D(Pose2D...waypoints) {
        this.waypoints = new ArrayList<>(Arrays.asList(waypoints));
        this.reversed = false;
    }

    public Path2D(boolean reversed, Pose2D...waypoints) {
        this.waypoints = new ArrayList<>(Arrays.asList(waypoints));
        this.reversed = reversed;
    }

    public ArrayList<Pose2D> getWaypoints() {
        return waypoints;
    }
    public Path2D setTangent(boolean t) {
        this.tangent = t;
        return this;
    }

    public Path2D setReversed(boolean r) {
        this.reversed = r;
        return this;
    }

    public boolean isTangent() {
        return tangent;
    }

    public double getLength() {
        // this will be used later. if we are close to our goal then we can just P2P i guess.
        // haha useless
        double sum = 0;
        for (int i = 0; i < waypoints.size()-1; i++) {
            sum += MathFunctions.getDistance(waypoints.get(i+1), waypoints.get(i));
        }
        return sum;
    }

    public int getSize() {
        return waypoints.size();
    }

    public Pose2D getPose(int i) {
        return waypoints.get(i);
    }

    public boolean isReversed() {
        return reversed;
    }

}
