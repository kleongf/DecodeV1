package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.controllers.FeedForwardController;
import org.firstinspires.ftc.teamcode.util.purepursuit.MathFunctions;

import static org.firstinspires.ftc.teamcode.robot.constants.RobotConstants.*;

public class Shooter extends Subsystem {
    private double targetVelocity = 0;
    public boolean shooterOn = false;
    private Servo latchServo;
    private Servo pitchServo;
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private FeedForwardController controller;
    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        latchServo = hardwareMap.get(Servo.class, "latchServo");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");

        controller = new FeedForwardController((1.0/2200), 0, 0.005);
    }
    @Override
    public void update() {
        if (shooterOn) {
            double power = controller.calculate(shooterMotor.getVelocity(), targetVelocity);
            shooterMotor.setPower(power);
            shooterMotor2.setPower(power);
        }
    }

    public void openLatch() {
        latchServo.setPosition(LATCH_OPEN);
    }

    public void closeLatch() {
        latchServo.setPosition(LATCH_CLOSED);
    }
    // note: in radians
    public void setShooterPitch(double angle) {
        double ticksPerRadian = (PITCH_SERVO_F-PITCH_SERVO_I)/(PITCH_F-PITCH_I);
        double pos = PITCH_SERVO_MIN + angle * ticksPerRadian;
        pitchServo.setPosition(MathFunctions.clamp(pos, PITCH_SERVO_I, PITCH_SERVO_F));
        // pitchServo.setPosition(PITCH_SERVO_MIN + angle * ticksPerRadian);
    }

    public void setTargetVelocity(double t) {
        targetVelocity = t;
    }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity();
    }

    public void setShooterOn(boolean x) {
        shooterOn = x;
    }

    public boolean atTarget(double threshold) {
        return Math.abs(shooterMotor.getVelocity()-targetVelocity) < threshold;
    }

    @Override
    public void start() {}
}
