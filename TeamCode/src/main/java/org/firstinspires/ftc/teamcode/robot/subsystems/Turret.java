package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.util.purepursuit.MathFunctions;

public class Turret extends Subsystem {
    public DcMotorEx turretMotor;
    public PIDFController turretController;
    public double target;
    private double ticksPerRevolution = 140; // 28*5 idk
    private double ticksPerRadian = ticksPerRevolution / (2 * Math.PI);
    ;
    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // tODO: dont always reset encoder
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretController = new PIDFController(0.1, 0, 0.001, 0);
        // might need feedforward
    }

    @Override
    public void update() {
        double current = turretMotor.getCurrentPosition() / ticksPerRadian;
        double power = -turretController.calculate(MathFunctions.angleWrap(current), MathFunctions.angleWrap(target));
        turretMotor.setPower(power);
    }

    @Override
    public void start() {

    }

    public void setTarget(double x) {
        target = x;
    }


}
