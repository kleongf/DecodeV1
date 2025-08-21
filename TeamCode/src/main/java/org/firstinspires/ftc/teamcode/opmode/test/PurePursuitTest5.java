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
import org.firstinspires.ftc.teamcode.util.purepursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.util.purepursuit.PurePursuitFollower;

@Autonomous(name = "pure pursuit test 5: pid to point TURBO EDITION")
public class PurePursuitTest5 extends OpMode {
    private PurePursuit follower;
    private StateMachine stateMachine;
    private final Pose startPose = new Pose(9, 40, Math.toRadians(0));
    private final Pose2D pose1 = new Pose2D(15, 35, Math.toRadians(30));
    private final Pose2D pose2 = new Pose2D(17, 43, Math.toRadians(-30));
    private final Pose2D pose3 = new Pose2D(9, 40, Math.toRadians(0));

    @Override
    public void init() {
        follower = new PurePursuit(hardwareMap);
        follower.setStartingPose(startPose);
        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> follower.pidToPoint(pose1))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.pidToPoint(pose2))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.pidToPoint(pose3))
                        .transition(new Transition(() -> !follower.isBusy()))
        );
    }
    @Override
    public void loop() {
        follower.update();
        stateMachine.update();
        telemetry.addLine("The robot should be able to pid to point regardless of velocity.");
        if (follower.currentPose != null) {
            telemetry.addLine("Pose: X: " + follower.currentPose.getX() + " Y: " + follower.currentPose.getY());
        }
        telemetry.update();
    }

    @Override
    public void start() {
        stateMachine.start();
    }
}
