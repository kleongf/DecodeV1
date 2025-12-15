package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.misc.Subsystem;

public class Pivot extends Subsystem {
    public CRServo pivotServo1;
    public CRServo pivotServo2;
    public Pivot(HardwareMap hardwareMap) {
        pivotServo1 = hardwareMap.get(CRServo.class, "pivotServo1");
        pivotServo2 = hardwareMap.get(CRServo.class, "pivotServo2");
        pivotServo1.setDirection(DcMotorSimple.Direction.FORWARD);
        pivotServo2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void update() {

    }

    @Override
    public void start() {

    }

    public void setPower(double power) {
        pivotServo1.setPower(power);
        pivotServo2.setPower(power);
    }
}
