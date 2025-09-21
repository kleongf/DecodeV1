package org.firstinspires.ftc.teamcode.robot.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.BetterIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.BetterShooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

import java.util.ArrayList;

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
                            shooter.setShooterOn(false);
                            intake.setIntakeOn(true);
                            intake.intakeDown();
                            shooter.closeLatch();
                        })
                        .maxTime(500)
        );
        commands.add(prepareIntake);

        prepareShooting = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.setIntakeOn(false);
                            shooter.setShooterOn(true);
                            intake.intakeLock();
                            shooter.closeLatch();
                            // TODO: placeholder for now, will be removed
                            shooter.setTargetVelocity(2000);
                        })
                        .maxTime(500)
        );
        commands.add(prepareShooting);

        startShooting = new StateMachine(
                new State()
                        .onEnter(() -> {
                            // saying everything so that in the event of a back button, we can do to prev state and run it
                            intake.setIntakeOn(true);;
                            shooter.setShooterOn(true);
                            shooter.openLatch();
                            intake.intakeLock();
                        })
                        .maxTime(500)
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
}

