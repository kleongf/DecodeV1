package org.firstinspires.ftc.teamcode.robot.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.BetterIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.BetterShooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

import java.util.ArrayList;

public class AutonomousRobotV1 {
    public boolean isBusy = false;
    private final ArrayList<Subsystem> subsystems;
    private final BulkRead bulkRead;
    public final BetterIntake intake;
    public final BetterShooter shooter;
    public final Turret turret;

    private final ArrayList<StateMachine> commands;
    public StateMachine prepareIntake;
    public StateMachine prepareShooting;
    public StateMachine slowIntake;
    public StateMachine startShooting;

    public AutonomousRobotV1(HardwareMap hardwareMap) {
        subsystems = new ArrayList<>();
        bulkRead = new BulkRead(hardwareMap);
        subsystems.add(bulkRead);
        intake = new BetterIntake(hardwareMap);
        subsystems.add(intake);
        shooter = new BetterShooter(hardwareMap);
        subsystems.add(shooter);
        turret = new Turret(hardwareMap);
        turret.resetEncoder();
        subsystems.add(turret);


        commands = new ArrayList<>();
        // prepares to intake: turns intake on and puts it down
        prepareIntake = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.setIntakeOn(true);
                            intake.state = BetterIntake.IntakeState.INTAKE_FAST;
                            intake.intakeDown();
                            shooter.closeLatch();
                        })
                        .maxTime(100)
        );
        commands.add(prepareIntake);
        // prepares to shoot by locking intake
        prepareShooting = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = BetterIntake.IntakeState.INTAKE_FAST;
                            intake.setIntakeOn(false);
                            intake.intakePushMid();
                            shooter.closeLatch();
                        })
                        .maxTime(10)
        );
        commands.add(prepareShooting);

        slowIntake = new StateMachine(
                new State()
                        .maxTime(350),
                new State()
                        .onEnter(() -> intake.state = BetterIntake.IntakeState.INTAKE_SLOW)
                        .maxTime(50)
        );
        commands.add(slowIntake);


        startShooting = new StateMachine(
                new State()
                        .onEnter(() -> {
                            // ??? saying everything so that in the event of a back button, we can do to prev state and run it
                            intake.state = BetterIntake.IntakeState.INTAKE_OFF;
                            intake.intakePushMid();
                        })
                        .maxTime(200),
                new State()
                        .onEnter(() -> {shooter.openLatch();})
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            intake.setIntakeOn(true);
                            intake.state = BetterIntake.IntakeState.INTAKE_FAST;})
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            intake.setIntakeOn(true);
                            intake.intakePush();
                        })
                        .maxTime(500),
                new State()
                        .onEnter(() -> {
                            intake.setIntakeOn(false);
                            intake.state = BetterIntake.IntakeState.INTAKE_OFF;
                            intake.intakeDown();
                        })
                        .maxTime(10)
        );
        commands.add(startShooting);
    }



    public void initPositions() {
        shooter.closeLatch();
        shooter.setTargetVelocity(0);
        shooter.setShooterPitch(Math.toRadians(0));
        shooter.setShooterOn(true);
        intake.setIntakeOn(false);
        intake.intakePushMid();
        turret.setTarget(0);
        intake.rightGateMid();
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
