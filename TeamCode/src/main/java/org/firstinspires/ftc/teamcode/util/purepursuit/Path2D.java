package org.firstinspires.ftc.teamcode.util.purepursuit;

import java.util.ArrayList;
import java.util.Arrays;

public class Path2D {
    private final ArrayList<Pose2D> waypoints;
    private boolean reversed;
    private boolean tangent = true;
    private double lookAheadDistance = PurePursuitConstants.LOOK_AHEAD_DISTANCE;
    private double maxPower = 1.0;

    public Path2D(Pose2D...waypoints) {
        this.waypoints = new ArrayList<>(Arrays.asList(waypoints));
        this.reversed = false;
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

    public Path2D setMaxPower(double m) {
        this.maxPower = m;
        return this;
    }

    public boolean isTangent() {return tangent;}

    public int getSize() {
        return waypoints.size();
    }

    public Pose2D getPose(int i) {
        return waypoints.get(i);
    }

    public boolean isReversed() {
        return reversed;
    }
    public double getMaxPower() {return maxPower;}
    public double getLookAheadDistance() {return lookAheadDistance;}

}
