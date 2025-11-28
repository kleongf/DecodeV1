package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;

@Config
@TeleOp(name="Position Tuner")
public class PositionTuner extends OpMode {
    public Servo leftLatch;
    public Servo pitchServo;
    public DcMotorEx intakeMotor;
    public static double latchTarget = RobotConstants.LATCH_CLOSED;
    public static double intakePower = 0;
    public static double pitchTarget = RobotConstants.PITCH_SERVO_MIN;

    @Override
    public void init() {
        leftLatch = hardwareMap.get(Servo.class, "latchServo");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");

        leftLatch.setDirection(Servo.Direction.FORWARD);
        pitchServo.setDirection(Servo.Direction.FORWARD);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        intakeMotor.setPower(intakePower);
       leftLatch.setPosition(latchTarget);
       pitchServo.setPosition(pitchTarget);
    }
}
