package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter extends Subsystem {
    // we gonna pid to velocity later
    private double targetVelocity = 0;
    private boolean shooterOn = false;
    // we might make this two cr servos with gecko wheels
    private Servo shooterServo;
    private DcMotorEx shooterMotorTop;
    private DcMotorEx shooterMotorBottom;
    public Shooter(HardwareMap hardwareMap) {
        shooterMotorTop = hardwareMap.get(DcMotorEx.class, "shooterMotorTop");
        shooterMotorTop.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorTop.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotorBottom = hardwareMap.get(DcMotorEx.class, "shooterMotorBottom");
        shooterMotorBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorBottom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(0);
    }
    @Override
    public void update() {
        if (shooterOn) {
            // later we will do velocity control with a pid
            shooterMotorTop.setPower(1);
            shooterMotorBottom.setPower(1);
        }
    }

    @Override
    public void start() {}
}
