package org.firstinspires.ftc.teamcode.opmode.test;

import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
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

@Autonomous(name = "pure pursuit test 2")
public class PurePursuitTest2 extends OpMode {
    private PurePursuitFollower follower;
    private StateMachine stateMachine;
    private final Pose startPose = new Pose(9, 40, Math.toRadians(0));
    private boolean followingForward = true;
    private final Path2D forwardPath = new Path2D(
            new Pose2D(9, 40, Math.toRadians(0)),
            new Pose2D(40, 25, Math.toRadians(-30)),
            new Pose2D(60, 10, Math.toRadians(-60))
    );

    private final Path2D backwardPath = new Path2D(
            true,
            new Pose2D(60, 10, Math.toRadians(-60)),
            new Pose2D(40, 25, Math.toRadians(-30)),
            new Pose2D(9, 40, Math.toRadians(0))
    );

    @Override
    public void init() {
        follower = new PurePursuitFollower(hardwareMap);
        follower.setStartingPose(startPose);
        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> follower.followPath(forwardPath))
                        .transition(new Transition(() -> follower.isFinished())),
                new State()
                        .onEnter(() -> {follower.followPath(backwardPath); followingForward = false;})
                        .transition(new Transition(() -> follower.isFinished()))
        );
    }
    @Override
    public void loop() {
        follower.update();
        stateMachine.update();
        telemetry.addData("Following forward path:", followingForward);
        telemetry.update();
    }

    @Override
    public void start() {
        stateMachine.start();
    }
}
