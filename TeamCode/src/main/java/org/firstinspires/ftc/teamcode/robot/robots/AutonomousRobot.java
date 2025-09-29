package org.firstinspires.ftc.teamcode.robot.robots;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.BetterIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.BetterShooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstantsAuto.*;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

import java.util.ArrayList;

public class AutonomousRobot {
    public boolean isBusy = false;
    private final ArrayList<Subsystem> subsystems;
    private final BulkRead bulkRead;
    private final BetterIntake intake;
    public final BetterShooter shooter;
    public final Turret turret;

    private final ArrayList<StateMachine> commands;
    public StateMachine prepareIntake;
    public StateMachine prepareShooting;
    public StateMachine startShooting;

    // button: start and stop intaking
    // shooter always spinning
    // button: releases the latch, starts spinning intake (since thats what powers it up)

    public AutonomousRobot(HardwareMap hardwareMap) {
        subsystems = new ArrayList<>();
        bulkRead = new BulkRead(hardwareMap);
        // TODO: for other subsystems in teleop just don't reset encoders
        subsystems.add(bulkRead);
        intake = new BetterIntake(hardwareMap);
        subsystems.add(intake);
        shooter = new BetterShooter(hardwareMap);
        subsystems.add(shooter);
        turret = new Turret(hardwareMap);
        subsystems.add(turret);


        commands = new ArrayList<>();
        prepareIntake = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.setIntakeOn(true);
                            intake.intakeDown();
                            shooter.closeLatch();
                        })
                        .maxTime(300)
        );
        commands.add(prepareIntake);

        prepareShooting = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.setIntakeOn(false);
                            intake.intakeLock();
                            shooter.closeLatch();
                        })
                        .maxTime(300)
        );
        commands.add(prepareShooting);

        startShooting = new StateMachine(
                new State()
                        .onEnter(shooter::openLatch)
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            intake.setIntakeOn(true);
                            intake.intakeLock();
                        })
                        .maxTime(1000)
        );
        commands.add(startShooting);
    }

    public void initPositions() {
        shooter.closeLatch();
        shooter.setTargetVelocity(0);
        shooter.setShooterPitch(Math.toRadians(30));
        shooter.setShooterOn(true);
        intake.setIntakeOn(false);
        intake.intakeLock();
        turret.setTarget(0);
    }

    public void update() {
        for (Subsystem subsystem: subsystems) {
            subsystem.update();
        }
        for (StateMachine command: commands) {
            command.update();
        }
    }

    public void start() {
        for (Subsystem subsystem: subsystems) {
            subsystem.start();
        }
    }
}