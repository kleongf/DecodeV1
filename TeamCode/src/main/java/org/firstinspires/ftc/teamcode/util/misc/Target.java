package org.firstinspires.ftc.teamcode.util.misc;

public class Target {
    private double xPos;
    private double yPos;
    private double heading;
    private double shooterVelocity;
    private double shooterPitch;
    public Target(double xPos, double yPos, double heading, double shooterVelocity, double shooterPitch) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.heading = heading;
        this.shooterVelocity = shooterVelocity;
        this.shooterPitch = shooterPitch;
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

    public double getShooterPitch() {
        return shooterPitch;
    }
}
