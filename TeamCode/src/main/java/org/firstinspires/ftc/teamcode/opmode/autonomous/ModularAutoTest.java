package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmode.teleop.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.misc.SOTM;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;

@Autonomous(name="Modular Autonomous Test", group="testing")
public class ModularAutoTest extends OpMode {
    private VoltageCompFollower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private final Pose startPose = PoseConstants.BLUE_CLOSE_AUTO_POSE;
    private Pose shootPose = new Pose(60, 84, Math.toRadians(180));
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private StateMachine preloadClose, firstSpike, secondSpike, thirdSpike;

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap, Alliance.BLUE);
        sotm2 = new SOTM(goalPose);

        preloadClose = robot.preloadClose(Alliance.BLUE, follower, startPose, shootPose);
        firstSpike = robot.firstSpikeMark(Alliance.BLUE, follower, shootPose, shootPose);
        secondSpike = robot.secondSpikeMark(Alliance.BLUE, follower, shootPose, shootPose);
        thirdSpike = robot.thirdSpikeMark(Alliance.BLUE, follower, shootPose, shootPose);

        stateMachine = new StateMachine(
                new State()
                    .onEnter(() -> preloadClose.start())
                    .transition(new Transition(() -> preloadClose.isFinished())),
                new State()
                        .onEnter(() -> firstSpike.start())
                        .transition(new Transition(() -> firstSpike.isFinished())),
                new State()
                        .onEnter(() -> secondSpike.start())
                        .transition(new Transition(() -> secondSpike.isFinished())),
                new State()
                        .onEnter(() -> thirdSpike.start())
                        .transition(new Transition(() -> thirdSpike.isFinished()))
        );

        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.initPositions();
    }
    @Override
    public void loop() {
        follower.update();
        double[] values = sotm2.calculateAzimuthThetaVelocity(follower.getPose(), new Vector());
        robot.setAzimuthThetaVelocity(values);

        preloadClose.update();
        firstSpike.update();
        secondSpike.update();
        thirdSpike.update();

        stateMachine.update();

        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        robot.shooter.state = Shooter.ShooterState.SHOOTER_ON;
        stateMachine.start();
        robot.start();
    }

    @Override
    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
    }
}
