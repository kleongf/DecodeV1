package org.firstinspires.ftc.teamcode.robot.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

import java.util.ArrayList;

public class TeleopRobot {
    public boolean isBusy = false;
    private final ArrayList<Subsystem> subsystems;
    private final BulkRead bulkRead;
    private final Intake intake;
    public final Shooter shooter;

    private final ArrayList<StateMachine> commands;
    public StateMachine prepareIntake;
    public StateMachine prepareShooting;
    public StateMachine shootGreen;
    public StateMachine shootPurple;

    public TeleopRobot(HardwareMap hardwareMap) {
        subsystems = new ArrayList<>();
        bulkRead = new BulkRead(hardwareMap);
        // TODO: for other subsystems in teleop just don't reset encoders
        subsystems.add(bulkRead);
        intake = new Intake(hardwareMap);
        subsystems.add(intake);
        shooter = new Shooter(hardwareMap);
        subsystems.add(shooter);

        commands = new ArrayList<>();
        prepareIntake = new StateMachine(
                new State()
                        .onEnter(() -> {
                            shooter.shooterOn = false;
                            intake.intakeOn = true;
                            intake.reset();
                            intake.intakeDown();
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
                        })
                        .maxTime(1000)
        );
        commands.add(prepareShooting);

        shootGreen = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.release("g");
                            intake.intakeOn = true;
                        })
                        .maxTime(1000),
                new State()
                        .onEnter(() -> {
                            intake.intakeOn = false;
                            shooter.launch();
                        })
                        .maxTime(500),
                new State()
                        .onEnter(() -> {
                            shooter.reset();
                        })
                        .maxTime(300)
        );
        commands.add(shootGreen);

        shootPurple = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.release("p");
                            intake.intakeOn = true;
                        })
                        .maxTime(1000),
                new State()
                        .onEnter(() -> {
                            intake.intakeOn = false;
                            shooter.launch();
                        })
                        .maxTime(500),
                new State()
                        .onEnter(() -> {
                            shooter.reset();
                        })
                        .maxTime(300)
        );
        commands.add(shootPurple);
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
