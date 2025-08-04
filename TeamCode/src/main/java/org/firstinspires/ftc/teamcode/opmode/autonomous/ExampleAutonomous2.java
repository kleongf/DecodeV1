package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.util.pathplanner.SafePathGenerator;
import org.firstinspires.ftc.teamcode.util.pathplanner.TargetPose;

public class ExampleAutonomous2 extends OpMode {
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SafePathGenerator pathGenerator;
    private final Pose startPose = new Pose(9, 66, Math.toRadians(0));

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap);
        pathGenerator = new SafePathGenerator(
                follower,
                new TargetPose(9, 66, Math.toRadians(0.0)),
                new TargetPose(40, 66, Math.toRadians(0.0)),
                new TargetPose(20, 18, Math.toRadians(0.0))
        );
        pathGenerator.generatePaths();
        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> pathGenerator.followNextPath(true))
                        .transition(new Transition(() -> !follower.isBusy()))
        ); // we would add states in here...

        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.initPositions();
    }
    @Override
    public void loop() {
        stateMachine.update();
        follower.update();
        robot.update();
    }

    @Override
    public void start() {
        stateMachine.start();
        robot.start();
    }

    @Override
    public void init_loop() {
        robot.update();
    }
}
