package org.firstinspires.ftc.teamcode.util.purepursuit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.purepursuit.LocalizerConstants.*;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.NanoTimer;
import java.util.Objects;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Localizer {
    private HardwareMap hardwareMap;
    private GoBildaPinpointDriver odo;
    private double previousHeading;
    private double totalHeading;
    private Pose startPose;
    private long deltaTimeNano;
    private NanoTimer timer;
    private Pose currentVelocity;
    private Pose pinpointPose;
    private boolean pinpointCooked;

    public Localizer(HardwareMap map) {
        this(map, new Pose());
    }

    public Localizer(HardwareMap map, Pose setStartPose) {
        this.pinpointCooked = false;
        this.hardwareMap = map;
        this.odo = (GoBildaPinpointDriver)this.hardwareMap.get(GoBildaPinpointDriver.class, LocalizerConstants.hardwareMapName);
        this.setOffsets(LocalizerConstants.forwardY, LocalizerConstants.strafeX, LocalizerConstants.distanceUnit);
        if (LocalizerConstants.useYawScalar) {
            this.odo.setYawScalar(LocalizerConstants.yawScalar);
        }

        if (LocalizerConstants.useCustomEncoderResolution) {
            this.odo.setEncoderResolution(LocalizerConstants.customEncoderResolution);
        } else {
            this.odo.setEncoderResolution(LocalizerConstants.encoderResolution);
        }

        this.odo.setEncoderDirections(LocalizerConstants.forwardEncoderDirection, LocalizerConstants.strafeEncoderDirection);
        this.resetPinpoint();
        this.setStartPose(setStartPose);
        this.totalHeading = (double)0.0F;
        this.timer = new NanoTimer();
        this.pinpointPose = this.startPose;
        this.currentVelocity = new Pose();
        this.deltaTimeNano = 1L;
        this.previousHeading = setStartPose.getHeading();
    }

    public Pose getPose() {
        return this.pinpointPose.copy();
    }

    public Pose2D getPose2D() {
        return new Pose2D(this.pinpointPose.getX(), this.pinpointPose.getY(), this.pinpointPose.getHeading());
    }

    public Pose getVelocity() {
        return this.currentVelocity.copy();
    }

    public Vector getVelocityVector() {
        return this.currentVelocity.getVector();
    }

    public double getSpeed() {
        return Math.sqrt(Math.pow(this.currentVelocity.getX(), 2) + Math.pow(this.currentVelocity.getY(), 2));
    }

    public void setStartPose(Pose setStart) {
        if (!Objects.equals(this.startPose, new Pose()) && this.startPose != null) {
            Pose currentPose = MathFunctions.subtractPoses(MathFunctions.rotatePose(this.pinpointPose, -this.startPose.getHeading(), false), this.startPose);
            this.setPose(MathFunctions.addPoses(setStart, MathFunctions.rotatePose(currentPose, setStart.getHeading(), false)));
        } else {
            this.setPose(setStart);
        }

        this.startPose = setStart;
    }

    public void setPose(Pose setPose) {
        this.odo.setPosition(new Pose(setPose.getX(), setPose.getY(), setPose.getHeading()));
        this.pinpointPose = setPose;
        this.previousHeading = setPose.getHeading();
    }

    public void update() {
        this.deltaTimeNano = this.timer.getElapsedTime();
        this.timer.resetTimer();
        this.odo.update();
        Pose currentPinpointPose = this.getPoseEstimate(this.odo.getPosition(), this.pinpointPose, this.deltaTimeNano);
        this.totalHeading += MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), this.previousHeading);
        this.previousHeading = currentPinpointPose.getHeading();
        Pose deltaPose = MathFunctions.subtractPoses(currentPinpointPose, this.pinpointPose);
        this.currentVelocity = new Pose(deltaPose.getX() / ((double)this.deltaTimeNano / Math.pow((double)10.0F, (double)9.0F)), deltaPose.getY() / ((double)this.deltaTimeNano / Math.pow((double)10.0F, (double)9.0F)), deltaPose.getHeading() / ((double)this.deltaTimeNano / Math.pow((double)10.0F, (double)9.0F)));
        this.pinpointPose = currentPinpointPose;
    }

    public double getTotalHeading() {
        return this.totalHeading;
    }

    public double getForwardMultiplier() {
        return (double)this.odo.getEncoderY();
    }

    public double getLateralMultiplier() {
        return (double)this.odo.getEncoderX();
    }

    public double getTurningMultiplier() {
        return (double)this.odo.getYawScalar();
    }

    private void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        this.odo.setOffsets(unit.toMm(xOffset), unit.toMm(yOffset));
    }

    public void resetIMU() {
        this.odo.recalibrateIMU();
    }

    private void resetPinpoint() {
        this.odo.resetPosAndIMU();

        try {
            Thread.sleep(300L);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private Pose getPoseEstimate(Pose pinpointEstimate, Pose currentPose, long deltaTime) {
        this.pinpointCooked = false;
        double x;
        if (!Double.isNaN(pinpointEstimate.getX())) {
            x = pinpointEstimate.getX();
        } else {
            x = currentPose.getX() + this.currentVelocity.getX() * (double)deltaTime / Math.pow((double)10.0F, (double)9.0F);
            this.pinpointCooked = true;
        }

        double y;
        if (!Double.isNaN(pinpointEstimate.getY())) {
            y = pinpointEstimate.getY();
        } else {
            y = currentPose.getY() + this.currentVelocity.getY() * (double)deltaTime / Math.pow((double)10.0F, (double)9.0F);
            this.pinpointCooked = true;
        }

        double heading;
        if (!Double.isNaN(pinpointEstimate.getHeading())) {
            heading = pinpointEstimate.getHeading();
        } else {
            heading = currentPose.getHeading() + this.currentVelocity.getHeading() * (double)deltaTime / Math.pow((double)10.0F, (double)9.0F);
            this.pinpointCooked = true;
        }

        return new Pose(x, y, heading);
    }

    public boolean isNAN() {
        return this.pinpointCooked;
    }
}

