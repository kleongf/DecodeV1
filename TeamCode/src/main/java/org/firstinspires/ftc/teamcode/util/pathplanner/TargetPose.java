package org.firstinspires.ftc.teamcode.util.pathplanner;

public class TargetPose {
    private final int x;
    private final int y;
    private final double heading;
    public TargetPose(int x, int y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }
}
