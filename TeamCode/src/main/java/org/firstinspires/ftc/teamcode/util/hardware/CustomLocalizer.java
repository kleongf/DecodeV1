package org.firstinspires.ftc.teamcode.util.hardware;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class CustomLocalizer {
    private GoBildaPinpointDriver odo;
    public static double forwardY = -4.35;
    public static double strafeX = -2.5197;
    public CustomLocalizer(HardwareMap hardwareMap, boolean resetPos) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(strafeX, forwardY, DistanceUnit.INCH); // replace with vals
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        if (resetPos) {
            odo.resetPosAndIMU();
        } else {
            odo.recalibrateIMU(); // is this really necessary? perhaps it is.
        }
    }

    public void setStartPose(Pose pose) {
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading()));
    }

    public Pose getPose() {
        Pose2D unconvertedPose = odo.getPosition();
        return new Pose(unconvertedPose.getX(DistanceUnit.INCH), unconvertedPose.getY(DistanceUnit.INCH), unconvertedPose.getHeading(AngleUnit.RADIANS));
    }

    public Vector getVelocity() {
        return new Vector(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH));
    }

    public void update() {
        odo.update();
    }

    public void setPose(Pose pose) {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading()));
    }
}
