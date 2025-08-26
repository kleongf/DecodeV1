package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class ActiveIntake extends Subsystem {
    private NormalizedColorSensor colorSensor;
    private DcMotorEx intakeMotor;
    private Servo latchServo;
    private String targetColor = "red";
    private boolean running = false;
    private double LATCH_CLOSED = 0;
    private double LATCH_OPEN = 0.5;
    public ActiveIntake(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        latchServo = hardwareMap.get(Servo.class, "latchServo");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private boolean inRange(double x, double lower, double upper) {
        return x <= upper && x >= lower;
    }

    public void setRunning(boolean x) {
        running = x;
    }

    public void setTargetColor(String color) {
        targetColor = color;
    }

    public String getTargetColor() {return targetColor;}

    private boolean isBlue(float[] values) {
        return inRange(values[0], 100, 130) && inRange(values[1], 100, 255) && inRange(values[2], 100, 255);
    }

    private boolean isYellow(float[] values) {
        return inRange(values[0], 20, 30) && inRange(values[1], 100, 255) && inRange(values[2], 100, 255);
    }

    private boolean isRed(float[] values) {
        return (inRange(values[0], 0, 10) && inRange(values[1], 100, 255) && inRange(values[2], 100, 255))
                ||
                (inRange(values[0], 160, 180) && inRange(values[1], 100, 255) && inRange(values[2], 100, 255));
    }

    @Override
    public void update() {
        float[] hsvValues = new float[3];
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        if (running) {
            if ((isRed(hsvValues) && targetColor.equals("red")) || (isBlue(hsvValues) && targetColor.equals("blue")) || (isYellow(hsvValues) && targetColor.equals("yellow"))) {
               latchServo.setPosition(LATCH_CLOSED);
               intakeMotor.setPower(0);
            } else {
                intakeMotor.setPower(1);
                latchServo.setPosition(LATCH_OPEN);
            }
        }
    }

    @Override
    public void start() {
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
    }
}
