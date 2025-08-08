package org.firstinspires.ftc.teamcode.util.purepursuit;

import com.pedropathing.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LocalizerConstants {
    public static String hardwareMapName = "pinpoint";
    public static double forwardY = -3.14961;
    public static double strafeX = 2.95276;
    public static DistanceUnit distanceUnit = DistanceUnit.INCH;
    public static boolean useYawScalar = false;
    public static double yawScalar = 1.0;
    public static boolean useCustomEncoderResolution = false;
    public static GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
    public static double customEncoderResolution = 13.26291192;
    public static GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
}
