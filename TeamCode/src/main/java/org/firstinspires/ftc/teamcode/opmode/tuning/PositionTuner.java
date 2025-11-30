package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.constants.RobotConstantsTele;
import org.firstinspires.ftc.teamcode.util.controllers.FeedForwardController;

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
    public Servo rightGateServo;
    public DcMotorEx intakeMotor;
    public DcMotorEx shooterMotor;
    public DcMotorEx shooterMotor2;
    public static double latchTarget = RobotConstantsTele.LATCH_CLOSED;
    public static double rightLatchTarget = 0;
    public static double centerLatchTarget = 0;
    public static double leftElbowTarget = RobotConstantsTele.INTAKE_DOWN;
    public static double rightElbowTarget = RobotConstantsTele.INTAKE_DOWN;
    public static double shooterTarget = 0;
    public static double shooterPower = 0;
    public static double intakePower = 0;
    public static double pitchTarget = RobotConstantsTele.PITCH_SERVO_MIN;
    public static double rightGateTarget = RobotConstantsTele.GATE_UP;

    private FeedForwardController controller;

    @Override
    public void init() {
        leftLatch = hardwareMap.get(Servo.class, "latchServo");
//        rightLatch = hardwareMap.get(Servo.class, "rightLatch");
//        centerLatch = hardwareMap.get(Servo.class, "centerLatch");

        rightGateServo = hardwareMap.get(Servo.class, "rightGateServo");

        leftElbow = hardwareMap.get(Servo.class, "leftElbow");
        // TODO: leftelbow down: 0.12, up = 0.4
        rightElbow = hardwareMap.get(Servo.class, "rightElbow");
//        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
//
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");



        leftLatch.setDirection(Servo.Direction.FORWARD);
//        rightLatch.setDirection(Servo.Direction.FORWARD);
//        centerLatch.setDirection(Servo.Direction.FORWARD);
        leftElbow.setDirection(Servo.Direction.FORWARD);
        rightElbow.setDirection(Servo.Direction.FORWARD);
//        shooterServo.setDirection(Servo.Direction.FORWARD);
        pitchServo.setDirection(Servo.Direction.FORWARD);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //controller = new FeedForwardController((1.0/2200), 0, 0.005);
    }

    @Override
    public void loop() {
        shooterMotor.setPower(shooterPower);
        shooterMotor2.setPower(shooterPower);


        intakeMotor.setPower(intakePower);
        leftLatch.setPosition(latchTarget);

        rightGateServo.setPosition(rightGateTarget);
//        rightLatch.setPosition(rightLatchTarget);
//        centerLatch.setPosition(centerLatchTarget);
        leftElbow.setPosition(leftElbowTarget);
        rightElbow.setPosition(rightElbowTarget);
//        shooterServo.setPosition(shooterTarget);
        pitchServo.setPosition(pitchTarget);
    }
}
