package org.firstinspires.ftc.teamcode.opmode.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.util.pathgenerator.ControlPoint;
import org.firstinspires.ftc.teamcode.util.pathgenerator.PathGenerator;
import org.firstinspires.ftc.teamcode.util.pathgenerator.TargetPose;
import org.firstinspires.ftc.teamcode.util.purepursuit.Path2D;
import org.firstinspires.ftc.teamcode.util.purepursuit.Pose2D;
import org.firstinspires.ftc.teamcode.util.purepursuit.PurePursuitFollower;

@Autonomous(name = "pedro pursuit test")
public class PedroTest extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(9, 40, Math.toRadians(0));
    private final PathChain forwardPath = new PathBuilder()
            .addPath(
            // Line 1
                        new BezierCurve(
                    new Point(9.000, 40,Point.CARTESIAN),
                                new Point(50, 30, Point.CARTESIAN),
                                new Point(60, 10, Point.CARTESIAN)
                        )
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-60))
            .setPathEndTValueConstraint(0.97)
            .build();

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
    }
    @Override
    public void loop() {
        follower.update();
    }

    @Override
    public void start() {
        follower.followPath(forwardPath, true);
    }
}
