package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.misc.Subsystem;

public class Intake extends Subsystem {
    public enum IntakeState {
        INTAKE_FAST,
        INTAKE_MEDIUM,
        INTAKE_SLOW,
        INTAKE_OFF
    }

    public IntakeState state = IntakeState.INTAKE_OFF;
    public DcMotorEx intakeMotor;
    private DigitalChannel top, middle, bottom;
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        top = hardwareMap.get(DigitalChannel.class, "topSensor");
//        middle = hardwareMap.get(DigitalChannel.class, "middleSensor");
//        bottom = hardwareMap.get(DigitalChannel.class, "bottomSensor");
    }

    @Override
    public void update() {
        switch (state) {
            case INTAKE_FAST:
                intakeMotor.setPower(1);
                break;
            case INTAKE_SLOW:
                intakeMotor.setPower(0.85);
                break;
            case INTAKE_OFF:
                intakeMotor.setPower(0);
                break;
        }
    }

    @Override
    public void start() {

    }
    // TODO: DO WHEN WE GET THE THE SENSOR
    public boolean intakeFull() {
        // return top.getState() && middle.getState() && bottom.getState();
        return false;
    }
}