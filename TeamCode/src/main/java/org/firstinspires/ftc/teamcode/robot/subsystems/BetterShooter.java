package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.controllers.TBHController;
import static org.firstinspires.ftc.teamcode.robot.constants.RobotConstantsTele.*;

public class BetterShooter extends Subsystem {
    private double targetVelocity = 0;
    private boolean isShooting = false;
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
            // if is shooting: actively shooting 3 balls
            if (isShooting && shooterMotor.getVelocity() < targetVelocity) {
                // max power unless less than target
                shooterMotor.setPower(1);
                return;
            }
            // if not less than target and/or shooting (we don't care)
            controller.setTarget(targetVelocity);
            double power = controller.calculate(shooterMotor.getVelocity());
            shooterMotor.setPower(power);
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
        // idk play with this
        double ticksPerRadian = (PITCH_SERVO_F-PITCH_SERVO_I)/(PITCH_F-PITCH_I);
        pitchServo.setPosition(PITCH_SERVO_MIN + angle * ticksPerRadian);
    }

    public void setTargetVelocity(double t) {
        targetVelocity = t;
    }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity();
    }

    @Override
    public void start() {}
}
