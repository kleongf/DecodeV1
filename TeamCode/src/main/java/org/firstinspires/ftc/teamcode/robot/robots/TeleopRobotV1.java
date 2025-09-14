package org.firstinspires.ftc.teamcode.robot.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.BetterIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.BetterShooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

import java.util.ArrayList;

public class TeleopRobotV1 {
    public boolean isBusy = false;
    private final ArrayList<Subsystem> subsystems;
    private final BulkRead bulkRead;
    private final BetterIntake intake;
    public final BetterShooter shooter;

    private final ArrayList<StateMachine> commands;
    public StateMachine prepareIntake;
    public StateMachine prepareShooting;
    public StateMachine startShooting;
    public StateMachine shootGreen;
    public StateMachine shootPurple;
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

        commands = new ArrayList<>();
        prepareIntake = new StateMachine(
                new State()
                        .onEnter(() -> {
                            shooter.shooterOn = false;
                            intake.intakeOn = true;
                            intake.intakeDown();
                            shooter.closeLatch();
                        })
                        .maxTime(1000)
        );
        commands.add(prepareIntake);

        prepareShooting = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.intakeOn = false;
                            intake.intakeUp();
                            shooter.shooterOn = true;
                            shooter.setTargetVelocity(2000);
                        })
                        .maxTime(1000)
        );
        commands.add(prepareShooting);

        startShooting = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.intakeOn = true;
                            shooter.openLatch();
                        })
                        .maxTime(1000)
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
}

