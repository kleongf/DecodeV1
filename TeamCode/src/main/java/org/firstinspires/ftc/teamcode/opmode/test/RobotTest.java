package org.firstinspires.ftc.teamcode.opmode.test;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.hardware.SmartGamepad;

@TeleOp(name="Robot Subsystem Test", group="comp")
public class RobotTest extends OpMode {
    private AutonomousRobot robot;
    private SmartGamepad gp1;
    private StateMachine testTurret;
    private StateMachine testShooter;
    private StateMachine testLatch;
    private StateMachine testIntake;

    @Override
    public void init() {
        robot = new AutonomousRobot(hardwareMap);
        gp1 = new SmartGamepad(gamepad1);

        testTurret = new StateMachine(
             new State()
                     .onEnter(() -> robot.turret.setTarget(-359))
                     .maxTime(3000),
                new State()
                        .onEnter(() -> robot.turret.setTarget(0))
                        .maxTime(3000)
        );

        testShooter = new StateMachine(
                new State()
                        .onEnter(() -> robot.shooter.setTargetVelocity(2500))
                        .maxTime(3000),
                new State()
                        .onEnter(() -> robot.shooter.setTargetVelocity(0))
                        .maxTime(3000)
        );

        testLatch = new StateMachine(
                new State()
                        .onEnter(() -> robot.shooter.openLatch())
                        .maxTime(2000),
                new State()
                        .onEnter(() -> robot.shooter.closeLatch())
                        .maxTime(2000)
        );


        testIntake = new StateMachine(
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_SLOW)
                        .maxTime(3000),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_FAST)
                        .maxTime(3000),
                new State()
                        .onEnter(() -> robot.intake.state = Intake.IntakeState.INTAKE_OFF)
                        .maxTime(3000)
        );
    }

    @Override
    public void loop() {
        gp1.update();

        if (gp1.aPressed()) {
            testTurret.start();
        }

        if (gp1.bPressed()) {
            testShooter.start();
        }

        if (gp1.xPressed()) {
            testLatch.start();
        }

        if (gp1.yPressed()) {
            testIntake.start();
        }

        testTurret.update();
        testShooter.update();
        testLatch.update();
        testIntake.update();

        telemetry.addData("shooter velocity", robot.shooter.getCurrentVelocity());
        telemetry.addData("intake velocity", robot.intake.intakeMotor.getVelocity());

        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        robot.initPositions();
        robot.shooter.setShooterOn(true);
        robot.start();
    }
}

