package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.controllers.TBHController;

public class BetterShooter extends Subsystem {
    private double targetVelocity = 0;
    private double radius = 1.45;
    private double targetAngle = Math.toRadians(30);
    public boolean shooterOn = false;
    private Servo latchServo;
    private Servo pitchServo;
    private DcMotorEx shooterMotor;
    private TBHController controller;
    public BetterShooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        latchServo = hardwareMap.get(Servo.class, "latchServo");

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        controller = new TBHController(0.00001, 1);
    }
    @Override
    public void update() {
        if (shooterOn) {
            controller.setTarget(targetVelocity);
            double power = controller.calculate(shooterMotor.getVelocity());
            shooterMotor.setPower(power);
        }
    }

    public void openLatch() {
        latchServo.setPosition(0.5);
    }

    public void closeLatch() {
        latchServo.setPosition(0.1);
    }

    public void setShooterPitch(double angle) {
        // idk play with this
        pitchServo.setPosition(0.1 + angle/Math.toRadians(90));
    }

    public void setTargetVelocity(double t) {
        targetVelocity = t;
    }

    public void setTargetVelocityInPerSec(double t) {
        // v = r * omega
        // omega = velocity (ticks) /
    }

    @Override
    public void start() {}
}
