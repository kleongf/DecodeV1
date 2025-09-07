package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends Subsystem {
    // check if color sensors will be useful
    public Servo leftLatch;
    public Servo rightLatch;
    public Servo centerLatch;
    public Servo leftElbow;
    public Servo rightElbow;
    public DcMotorEx intakeMotor;
    public boolean intakeOn;
    public Intake(HardwareMap hardwareMap) {
        leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        rightLatch = hardwareMap.get(Servo.class, "rightLatch");
        centerLatch = hardwareMap.get(Servo.class, "centerLatch");

        leftElbow = hardwareMap.get(Servo.class, "leftElbow");
        rightElbow = hardwareMap.get(Servo.class, "rightElbow");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update() {
        if (intakeOn) {
            intakeMotor.setPower(1);
        }
    }

    @Override
    public void start() {
        leftLatch.setPosition(0.1);
        rightLatch.setPosition(0.1);
        centerLatch.setPosition(0.1);
    }

    public void reset() {
        leftLatch.setPosition(0.1);
        rightLatch.setPosition(0.1);
        centerLatch.setPosition(0.1);
    }

    public void intakeUp() {
        leftElbow.setPosition(0.5);
        rightElbow.setPosition(0.5);
    }

    public void intakeDown() {
        leftElbow.setPosition(0.1);
        rightElbow.setPosition(0.1);
    }

    public void releaseLeft() {
        leftLatch.setPosition(0.5);
    }

    public void releaseRight() {
        rightLatch.setPosition(0.5);
    }

    public void releaseCenter() {
        centerLatch.setPosition(0.5);
    }
}
