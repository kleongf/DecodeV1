package org.firstinspires.ftc.teamcode.util.misc;

public class Target {
    private double xPos;
    private double yPos;
    private double heading;
    private double shooterVelocity;
    public Target(double xPos, double yPos, double heading, double shooterVelocity) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.heading = heading;
        this.shooterVelocity = shooterVelocity;
    }

    public double getxPos() {
        return xPos;
    }

    public double getyPos() {
        return yPos;
    }

    public double getShooterVelocity() {
        return shooterVelocity;
    }

    public double getHeading() {
        return heading;
    }
}
