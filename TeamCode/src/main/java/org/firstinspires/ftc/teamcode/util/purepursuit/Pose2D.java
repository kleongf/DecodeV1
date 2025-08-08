package org.firstinspires.ftc.teamcode.util.purepursuit;

public class Pose2D extends Point2D {
    private double heading;
    public Pose2D(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public double getHeading() {return heading;}
    public void setHeading(double h) {heading = h;}
}
