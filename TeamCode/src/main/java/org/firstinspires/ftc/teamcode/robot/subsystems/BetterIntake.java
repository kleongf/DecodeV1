package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.robot.constants.RobotConstantsTele.*;

public class BetterIntake extends Subsystem {
    public Servo leftElbow;
    public Servo rightElbow;
    public DcMotorEx intakeMotor;
    public boolean intakeOn;
    public BetterIntake(HardwareMap hardwareMap) {
        leftElbow = hardwareMap.get(Servo.class, "leftElbow");
        rightElbow = hardwareMap.get(Servo.class, "rightElbow");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        if (intakeOn) {
            intakeMotor.setPower(1);
        }
    }

    @Override
    public void start() {
        leftElbow.setPosition(INTAKE_DOWN);
        rightElbow.setPosition(INTAKE_DOWN);
    }

    public void intakeDown() {
        leftElbow.setPosition(INTAKE_DOWN);
        rightElbow.setPosition(INTAKE_DOWN);
    }
    // locks the balls?
    public void intakeLock() {
        leftElbow.setPosition(INTAKE_LOCK);
        rightElbow.setPosition(INTAKE_LOCK);
    }
}
