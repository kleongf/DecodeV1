package org.firstinspires.ftc.teamcode.util.purepursuit2;

import com.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.purepursuit2.constants.PPFollowerConstants;

import java.util.ArrayList;
import java.util.Arrays;

public class PPPath {
    private final ArrayList<Pose> waypoints;
    private boolean reversed;
    private boolean tangent = true;
    private double lookAheadDistance = PPFollowerConstants.LOOK_AHEAD_DISTANCE;
    private double maxVelocity = PPFollowerConstants.MAX_VELOCITY;
    private double maxAcceleration = PPFollowerConstants.MAX_ACCELERATION;
    private double pathEndDistanceConstraint = PPFollowerConstants.PATH_END_DISTANCE_CONSTRAINT;
    private double pathEndHeadingConstraint = PPFollowerConstants.PATH_END_HEADING_CONSTRAINT;
    private double pathEndSpeedConstraint = PPFollowerConstants.PATH_END_SPEED_CONSTRAINT;
    private boolean holdPoint = true;
    private double maxPower = 1.0;
    private double holdPointScaleFactor = PPFollowerConstants.HOLD_POINT_SCALE_FACTOR;

    public PPPath(Pose...waypoints) {
        this.waypoints = new ArrayList<>(Arrays.asList(waypoints));
        this.reversed = false;
    }

    public ArrayList<Pose> getWaypoints() {
        return waypoints;
    }
    public PPPath setTangent(boolean t) {
        this.tangent = t;
        return this;
    }

    public PPPath setReversed(boolean r) {
        this.reversed = r;
        return this;
    }

    public PPPath setLookAheadDistance(double l) {
        this.lookAheadDistance = l;
        return this;
    }

    public PPPath setMaxVelocity(double v) {
        this.maxVelocity = v;
        return this;
    }

    public PPPath setMaxAcceleration(double a) {
        this.maxAcceleration = a;
        return this;
    }

    public PPPath setPathEndDistanceConstraint(double x) {
        this.pathEndDistanceConstraint = x;
        return this;
    }

    public PPPath setPathEndHeadingConstraint(double x) {
        this.pathEndHeadingConstraint = x;
        return this;
    }

    public PPPath setPathEndSpeedConstraint(double x) {
        this.pathEndSpeedConstraint = x;
        return this;
    }

    public PPPath setHoldPoint(boolean x) {
        this.holdPoint = x;
        return this;
    }

    public PPPath setMaxPower(double x) {
        maxPower = x;
        return this;
    }

    public PPPath setHoldPointScaleFactor(double a) {
        this.holdPointScaleFactor = a;
        return this;
    }

    public boolean isTangent() {return tangent;}

    public int getSize() {
        return waypoints.size();
    }

    public Pose getPose(int i) {
        return waypoints.get(i);
    }

    public boolean isReversed() {
        return reversed;
    }
    public double getLookAheadDistance() {return lookAheadDistance;}
    public double getMaxVelocity() {return maxVelocity;}
    public double getMaxAcceleration() {return maxAcceleration;}
    public double getPathEndDistanceConstraint() {return pathEndDistanceConstraint;}
    public double getPathEndHeadingConstraint() {return pathEndHeadingConstraint;}
    public double getPathEndSpeedConstraint() {return pathEndSpeedConstraint;}

    public boolean getHoldPoint() {return holdPoint;}
    public double getMaxPower() {return maxPower;}
    public double getHoldPointScaleFactor() {return holdPointScaleFactor;}
}
