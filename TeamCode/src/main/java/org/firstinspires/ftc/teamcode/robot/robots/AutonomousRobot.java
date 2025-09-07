package org.firstinspires.ftc.teamcode.robot.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

import java.util.ArrayList;

public class AutonomousRobot {
    private final ArrayList<Subsystem> subsystems;
    public final BulkRead bulkRead;
    public final Vision vision;

    private final ArrayList<StateMachine> commands;
    public AutonomousRobot(HardwareMap hardwareMap) {
        subsystems = new ArrayList<>();

        bulkRead = new BulkRead(hardwareMap);
        subsystems.add(bulkRead);
        vision = new Vision(hardwareMap);
        subsystems.add(vision);

        commands = new ArrayList<>();
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
