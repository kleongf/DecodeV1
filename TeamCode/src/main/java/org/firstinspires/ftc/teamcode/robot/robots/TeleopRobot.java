package org.firstinspires.ftc.teamcode.robot.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

import java.util.ArrayList;

public class TeleopRobot {
    private final ArrayList<Subsystem> subsystems;
    private final BulkRead bulkRead;

    private final ArrayList<StateMachine> commands;
    public StateMachine scoreSpecimen;

    public TeleopRobot(HardwareMap hardwareMap) {
        subsystems = new ArrayList<>();
        bulkRead = new BulkRead(hardwareMap);
        // for other subsystems in teleop just don't reset encoders
        subsystems.add(bulkRead);

        commands = new ArrayList<>();
        scoreSpecimen = new StateMachine(
                new State()
                        .onEnter(() -> System.out.println("open claw lol"))
                        .maxTime(1000)
        );
        commands.add(scoreSpecimen);
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
