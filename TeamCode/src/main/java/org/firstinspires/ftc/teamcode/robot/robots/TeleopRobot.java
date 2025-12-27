package org.firstinspires.ftc.teamcode.robot.robots;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.util.misc.Subsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;

import java.util.ArrayList;

public class TeleopRobot {
    // new idea: how about we stay in the idle state when the thing is !isFinished? or something. need better method
    // or be able to call the stop multipossess on command, which is the main problem
    // intake slow is 0.8 while intake fast is 1 i dont think it really makes a difference
    private final ArrayList<Subsystem> subsystems;
    public final BulkRead bulkRead;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;
    public final LimelightLocalizer limelightLocalizer;
    public final Pivot pivot;

    private final ArrayList<StateMachine> commands;
    public StateMachine shootCommand, shootCommandSlow;
    public StateMachine idleCommand;

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

        limelightLocalizer = new LimelightLocalizer(hardwareMap);
        limelightLocalizer.setPipeline(LimelightLocalizer.Pipeline.APRILTAG);
        subsystems.add(limelightLocalizer);

        pivot = new Pivot(hardwareMap);
        subsystems.add(pivot);

        commands = new ArrayList<>();

        shootCommand = new StateMachine(
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
                        // TODO: .transition(new Transition(() -> !intake.intakeFull()))
                        // this does not quite work unless we know exactly how many we have
                        .maxTime(600),
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                            shooter.closeLatch();
                        })
                        .maxTime(100)
        );
        commands.add(shootCommand);

        shootCommandSlow = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_OFF;
                            shooter.openLatch();
                        })
                        .maxTime(150),
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_SLOW;
                        })
                        // TODO: .transition(new Transition(() -> !intake.intakeFull()))
                        // this does not quite work unless we know exactly how many we have
                        .maxTime(1200),
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_FAST;
                            shooter.closeLatch();
                        })
                        .maxTime(100)
        );

        commands.add(shootCommandSlow);

        idleCommand = new StateMachine(
                new State()
                        .onEnter(() -> {
                            intake.state = Intake.IntakeState.INTAKE_SLOW;
                            shooter.closeLatch();
                        })
                        .maxTime(100)
        );
        commands.add(idleCommand);
    }

    public void setAzimuthThetaVelocity(double[] values) {
        turret.setTarget(values[0]);
        shooter.setShooterPitch(values[1]);
        shooter.setTargetVelocity(values[2]);
    }

    public void initPositions() {
        // configure shooter
        shooter.state = Shooter.ShooterState.SHOOTER_ON;
        shooter.closeLatch();
        limelightLocalizer.setPipeline(LimelightLocalizer.Pipeline.APRILTAG);
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

    private boolean inFarLeftZone(Pose pose) {
        // left side, robot pose must be below line, it starts at -48 i think and goes to 24
        double zoneY = -48 + 1 * pose.getX();
        return pose.getX() >= 48 && pose.getX() <= 72 && pose.getY() < zoneY;
    }

    private boolean inFarRightZone(Pose pose) {
        // left side, robot pose must be below line, it starts at -48 i think and goes to 24
        double zoneY = 96 - 1 * pose.getX();
        return pose.getX() >= 72 && pose.getX() <= 144 && pose.getY() < zoneY;
    }
    public boolean inShootingZone(Pose pose) {
        return inRightZone(pose) || inLeftZone(pose) || inFarLeftZone(pose) || inFarRightZone(pose);
    }
}

