package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.robot.constants.RobotConstantsTele.*;

public class BetterIntake extends Subsystem {
    public enum IntakeState {
        INTAKE_FAST,
        INTAKE_SLOW,
        INTAKE_OFF
    }
    public IntakeState state = IntakeState.INTAKE_OFF;
    public Servo rightGateServo;
    public Servo leftGateServo;
    public Servo leftElbow;
    public Servo rightElbow;
    public DcMotorEx intakeMotor;
    private boolean intakeOn;
    public BetterIntake(HardwareMap hardwareMap) {
        leftElbow = hardwareMap.get(Servo.class, "leftElbow");
        rightElbow = hardwareMap.get(Servo.class, "rightElbow");

        rightGateServo = hardwareMap.get(Servo.class, "rightGateServo");
        //leftGateServo = hardwareMap.get(Servo.class, "leftGateServo");//todo: not until we get it installed

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        switch (state) {
            case INTAKE_FAST:
                intakeMotor.setPower(1);
                break;
            case INTAKE_SLOW:
                intakeMotor.setPower(0.5);
                break;
            case INTAKE_OFF:
                intakeMotor.setPower(0);
                break;
        }
//        if (intakeOn) {
//            intakeMotor.setPower(1);
//        } else {
//            intakeMotor.setPower(0);
//        }
    }

    @Override
    public void start() {
        leftElbow.setPosition(INTAKE_DOWN);
        rightElbow.setPosition(INTAKE_DOWN);

        rightGateMid();
    }

    public void intakeDown() {
        leftElbow.setPosition(INTAKE_DOWN);
        rightElbow.setPosition(INTAKE_DOWN);
    }
    // locks the balls?
    public void intakePush() {
        leftElbow.setPosition(INTAKE_PUSH);
        rightElbow.setPosition(INTAKE_PUSH);
    }
    public void intakeHigh(){
        leftElbow.setPosition(INTAKE_HIGH);
        rightElbow.setPosition(INTAKE_HIGH);
    }
    public void intakePushMid() {
        leftElbow.setPosition(INTAKE_PUSH_MID);
        rightElbow.setPosition(INTAKE_PUSH_MID);
    }
    public void rightGateUp(){
        rightGateServo.setPosition(GATE_UP);
    }
    public void rightGateDown(){
        rightGateServo.setPosition(GATE_DOWN);
    }
    public void rightGateMid(){
        rightGateServo.setPosition(GATE_MID);
    }
//    public void leftGateUp(){//todo: not yet
//        leftGateServo.setPosition(GATE_UP);
//    }
//    public void leftGateDown(){
//        leftGateServo.setPosition(GATE_DOWN);
//    }
    public void setIntakeOn(boolean x){
        intakeOn = x;
    }

    public boolean isIntakeOn() {
        return intakeOn;
    }
}
