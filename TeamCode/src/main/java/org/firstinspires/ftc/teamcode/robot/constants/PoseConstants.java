package org.firstinspires.ftc.teamcode.robot.constants;

import com.pedropathing.localization.Pose;

public class PoseConstants {
    // Teleop Poses
    public static Pose BLUE_GOAL_POSE = new Pose(0, 144, Math.toRadians(135));
    public static Pose RED_GOAL_POSE = new Pose(0, 144, Math.toRadians(45));
    public static Pose BLUE_FAR_POSE =  new Pose(54, 12, Math.toRadians(180));
    public static Pose RED_FAR_POSE =  new Pose(144-54, 12, Math.toRadians(0));
    public static Pose BLUE_GATE_POSE = new Pose(20, 68, Math.toRadians(180));

    public static Pose RED_GATE_POSE = new Pose(144-20, 68, Math.toRadians(0));

    // Autonomous Poses
    public static Pose BLUE_CLOSE_AUTO_POSE = new Pose(31, 137, Math.toRadians(270));
    public static Pose RED_CLOSE_AUTO_POSE = new Pose(144-31, 137, Math.toRadians(270));
    // TODO: rename to blue far auto start pose
    public static Pose BLUE_FAR_AUTO_POSE = new Pose(55, 6, Math.toRadians(90));
    public static Pose RED_FAR_AUTO_POSE = new Pose(144-55, 6, Math.toRadians(90));

    // TODO: FIND NEW GATE POSITIONS
    public static Pose BLUE_GATE_AUTO_POSE =  new Pose(12, 62.5, Math.toRadians(148.5));
    public static Pose RED_GATE_AUTO_POSE = new Pose(144-13, 62, Math.toRadians(180-142));
    public static Pose BLUE_SHOOT_AUTO_POSE = new Pose(60, 74, Math.toRadians(142));
    public static Pose RED_SHOOT_AUTO_POSE = new Pose(144-60, 74, Math.toRadians(142));
}
