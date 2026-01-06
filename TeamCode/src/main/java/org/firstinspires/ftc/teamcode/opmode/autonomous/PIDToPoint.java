package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.util.purepursuit2.TestFollower;

@Config
@Autonomous(name="pid to point test", group="not a comp")
public class PIDToPoint extends OpMode {
    private TestFollower follower;
    private final Pose startPose = new Pose(42.65,8,Math.toRadians(180));
    private final Pose goalPose = new Pose(9, 9, Math.toRadians(180));
    public static double goalX = 9;
    public static double goalY = 9;
    public static double goalHeadingDegrees = 180;
    public static boolean confirmPose = false;

    @Override
    public void init() {
        follower = new TestFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setGoalPose(goalPose);
    }
    @Override
    public void loop() {
        follower.update();
        telemetry.update();

        if (confirmPose) {
            follower.setGoalPose(new Pose(goalX, goalY, Math.toRadians(goalHeadingDegrees)));
        }
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {;
    }
}
