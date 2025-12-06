package org.firstinspires.ftc.teamcode.robot.robots;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

import java.util.ArrayList;

public class TeleopRobot {
    public boolean isBusy = false;
    private final ArrayList<Subsystem> subsystems;
    private final BulkRead bulkRead;
    private final Intake intake;
    public final Shooter shooter;
    public final Turret turret;

    private final ArrayList<StateMachine> commands;
    public StateMachine prepareIntake;
    public StateMachine prepareShooting;
    public StateMachine startShooting;

    public TeleopRobot(HardwareMap hardwareMap) {
        subsystems = new ArrayList<>();
        bulkRead = new BulkRead(hardwareMap);
        subsystems.add(bulkRead);
        intake = new Intake(hardwareMap);
        subsystems.add(intake);
        shooter = new Shooter(hardwareMap);
        subsystems.add(shooter);
        turret = new Turret(hardwareMap);
        subsystems.add(turret);

        commands = new ArrayList<>();
        // prepare to intake: turn on intake and close latch
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
                            shooter.openLatch();
                        })
                        .maxTime(150),
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                        })
                        .maxTime(600),
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_OFF;
                            shooter.closeLatch();
                        })
                        .maxTime(100)
        );
        commands.add(startShooting);

        // airSortShoot:
        // same start + speed up flywheel to first
        // move to position 1 (150 ms)
        // speed up+change flywheel (100 ms)
        // move to position 2 (150 ms)
        // repeat
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

    private boolean inLeftZone(Pose pose) {
        // right side. robot pose must be above the line with slope -1
        double zoneY = 144 + -1 * pose.getX();
        return pose.getX() < 72 && pose.getY() > zoneY;
    }

    private boolean inRightZone(Pose pose) {
        // left side. robot pose must be above line with slope 1
        double zoneY = 72 + 1 * (pose.getX()-72);
        return pose.getX() >= 72 && pose.getY() > zoneY;
    }
    public boolean inShootingZone(Pose pose) {
        return inRightZone(pose) || inLeftZone(pose);
    }
}

