package org.firstinspires.ftc.teamcode.robot.robots;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.BetterIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.BetterShooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

import java.util.ArrayList;
import java.util.Timer;

public class TeleopRobotV1 {
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
    //timers
    public Timer shootTimer;

    // button: start and stop intaking
    // shooter always spinning
    // button: releases the latch, starts spinning intake (since thats what powers it up)

    public TeleopRobotV1(HardwareMap hardwareMap) {
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

        // you know what I actually think that keeping the shooter on the entire time is a better idea
        // but we can playe with it

        commands = new ArrayList<>();
        prepareIntake = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.setIntakeOn(true);
                            intake.intakeDown();
                            shooter.closeLatch();
                        })
                        .maxTime(100)
        );
        commands.add(prepareIntake);
        // prepare to shoot by locking intake
        prepareShooting = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.setIntakeOn(false);
                            intake.intakePushMid();
                            shooter.closeLatch();
                        })
                        .maxTime(100)
        );
        commands.add(prepareShooting);

        shootTimer = new Timer();

        startShooting = new StateMachine(
                new State()
                        .onEnter(() -> {
                            // ??? saying everything so that in the event of a back button, we can do to prev state and run it
                            shooter.openLatch();
                            intake.setIntakeOn(true);
                            intake.intakePushMid();
                        })
                        .maxTime(500),
                new State()
                        .onEnter(() -> {
                            shooter.openLatch();
                            intake.setIntakeOn(true);
                            intake.intakePush();

                        })
                        .maxTime(500),
                new State()
                        .onEnter(() -> {
                            intake.setIntakeOn(false);
                            intake.intakeDown();
                        })
                        .maxTime(100)
        );
        commands.add(startShooting);
    }

    public void initPositions() {

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

    public boolean isBusy() {
        return isBusy;
    }
    // future functions to help with automatic shooting if we fit constraints

    private boolean inRightZone(Pose pose) {
        // right side. robot pose must be above the line with slope -1
        double zoneY = 144 + -1 * pose.getX();
        return pose.getX() < 72 && pose.getY() > zoneY;
    }

    private boolean inLeftZone(Pose pose) {
        // left side. robot pose must be above line with slope 1
        double zoneY = 72 + 1 * (pose.getX()-72);
        return pose.getX() >= 72 && pose.getY() > zoneY;
    }

    public boolean inShootingZone(Pose pose) {
        return inRightZone(pose) || inLeftZone(pose);
    }
}

