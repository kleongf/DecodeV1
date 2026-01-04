package org.firstinspires.ftc.teamcode.util.purepursuit2.tuners;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmode.teleop.Alliance;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.SOTM;
import org.firstinspires.ftc.teamcode.util.purepursuit2.PPFollower;
import org.firstinspires.ftc.teamcode.util.purepursuit2.PPPath;

@Config
@Autonomous(name="straight back and forth", group="not a comp")
public class StraightBackAndForth extends OpMode {
    private PPFollower follower;
    public static double kP_lateral = 0;
    public static double kD_lateral = 0;
    public static double kP_longitudinal = 0;
    public static double kD_longitudinal = 0;
    public static double kQ_lateral = 0;
    public static double kQ_longitudinal = 0;
    public static double kV_lateral = 0;
    public static double kV_longitudinal = 0;
    public static double kV_heading = 0;
    private boolean forward = true;

    private PPPath intakeCorner, shootCorner, intakeThird, shootThird, intakePile1, shootPile1, intakePile2, shootPile2, intakePile3, shootPile3, intakePile4, shootPile4, intakePile5, shootPile5, intakePile6, shootPile6, intakePile7, shootPile7, intakePile8, shootPile8;
    public void buildPaths() {
        intakeCorner = new PPPath(
                new Pose(42.65000, 8.000, Math.toRadians(180)),
                new Pose(9.000, 9.000, Math.toRadians(180))
        ).setTangent(false);

        shootCorner = new PPPath(
                new Pose(9.000, 9.000, Math.toRadians(180)),
                new Pose(56,20, Math.toRadians(180))
        ).setTangent(false);

        intakeThird = new PPPath(
                new Pose(56,20, Math.toRadians(180)),
                new Pose(44.000, 36.000, Math.toRadians(180)),
                new Pose(13.000, 36.000, Math.toRadians(180))
        ).setTangent(false);

        shootThird = new PPPath(
                new Pose(13.000, 36.000, Math.toRadians(180)),
                new Pose(56, 20, Math.toRadians(180))
        ).setTangent(false);

    }

    @Override
    public void init() {
        follower = new PPFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void loop() {

        if (forward) {
            if (!follower.isBusy()) {
                forward = false;
                follower.followPath(
                        new PPPath(
                                new Pose(0, 48, Math.toRadians(90)),
                                new Pose(0, 0, Math.toRadians(90))
                        ).setTangent(false)
                );
            }
        } else {
            if (!follower.isBusy()) {
                forward = true;
                follower.followPath(
                        new PPPath(
                                new Pose(0, 0, Math.toRadians(90)),
                                new Pose(0, 48, Math.toRadians(90))
                        ).setTangent(false)
                );
            }
        }
        follower.update();
        telemetry.update();
    }

    @Override
    public void start() {

    }
}
