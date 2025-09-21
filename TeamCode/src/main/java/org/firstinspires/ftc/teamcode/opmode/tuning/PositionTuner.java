package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Position Tuner")
public class PositionTuner extends OpMode {
    public Servo leftLatch;
    public Servo rightLatch;
    public Servo centerLatch;
    public Servo leftElbow;
    public Servo rightElbow;
    public Servo shooterServo;
    public Servo pitchServo;
    public static double leftLatchTarget = 0;
    public static double rightLatchTarget = 0;
    public static double centerLatchTarget = 0;
    public static double leftElbowTarget = 0;
    public static double rightElbowTarget = 0;
    public static double shooterTarget = 0;
    public static double pitchTarget = 0.19;

    @Override
    public void init() {
//        leftLatch = hardwareMap.get(Servo.class, "leftLatch");
//        rightLatch = hardwareMap.get(Servo.class, "rightLatch");
//        centerLatch = hardwareMap.get(Servo.class, "centerLatch");

        leftElbow = hardwareMap.get(Servo.class, "leftElbow");
        // TODO: leftelbow down: 0.12, up = 0.4
        rightElbow = hardwareMap.get(Servo.class, "rightElbow");
//        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
//
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        // TODO: pitchServo down: 0.19, up ~ 0.7

//        leftLatch.setDirection(Servo.Direction.FORWARD);
//        rightLatch.setDirection(Servo.Direction.FORWARD);
//        centerLatch.setDirection(Servo.Direction.FORWARD);
        leftElbow.setDirection(Servo.Direction.FORWARD);
        rightElbow.setDirection(Servo.Direction.FORWARD);
//        shooterServo.setDirection(Servo.Direction.FORWARD);
        pitchServo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
//        leftLatch.setPosition(leftLatchTarget);
//        rightLatch.setPosition(rightLatchTarget);
//        centerLatch.setPosition(centerLatchTarget);
        leftElbow.setPosition(leftElbowTarget);
        rightElbow.setPosition(rightElbowTarget);
//        shooterServo.setPosition(shooterTarget);
        pitchServo.setPosition(pitchTarget);
    }
}
