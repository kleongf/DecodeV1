package org.firstinspires.ftc.teamcode.robot.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

import java.util.ArrayList;

public class AutonomousRobot {
    public boolean isBusy = false;
    private final ArrayList<Subsystem> subsystems;
    private final BulkRead bulkRead;
    public final Intake intake;
    public final Shooter shooter;
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
        intake = new Intake(hardwareMap);
        subsystems.add(intake);
        shooter = new Shooter(hardwareMap);
        subsystems.add(shooter);
        turret = new Turret(hardwareMap);
        turret.resetEncoder();
        subsystems.add(turret);


        commands = new ArrayList<>();
        prepareIntake = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                            shooter.closeLatch();
                        })
                        .maxTime(100)
        );
        commands.add(prepareIntake);
        // prepare to shoot by slowing down intake
        prepareShooting = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_SLOW;
                        })
                        .maxTime(100)
        );
        commands.add(prepareShooting);

        // shoots: stops intake first, then turns it on
        startShooting = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_OFF;
                        })
                        .maxTime(200),
                new State()
                        .onEnter(() -> {
                            shooter.openLatch();
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                        })
                        .maxTime(600)
        );
        commands.add(startShooting);
    }

    public void setAzimuthThetaVelocity(double[] values) {
        turret.setTarget(values[0]);
        shooter.setShooterPitch(values[1]);
        shooter.setTargetVelocity(values[2]);
    }

    public void initPositions() {
        shooter.closeLatch();
        shooter.setTargetVelocity(0);
        shooter.setShooterPitch(Math.toRadians(0));
        shooter.setShooterOn(true);
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