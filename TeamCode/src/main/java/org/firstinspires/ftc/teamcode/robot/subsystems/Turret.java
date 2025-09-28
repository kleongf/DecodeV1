package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.controllers.HeadingPIDFController;
import org.firstinspires.ftc.teamcode.util.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.util.purepursuit.MathFunctions;

public class Turret extends Subsystem {
    public DcMotorEx turretMotor;
    public PIDFController turretController;
    public double target = 0;
    private double ticksPerRevolution = 1918; // 383.6*5 idk
    private double ticksPerRadian = ticksPerRevolution / (2 * Math.PI);
    ;
    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // tODO: dont always reset encoder
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // i changed this to heading pidf controller

        turretController = new PIDFController(2, 0, 0.06, 0);
        // might need feedforward
    }

    @Override
    public void update() {
        // i need to do something so it doesn't overdo it or make a 360.
        // was not anglewrapped before
        double current = turretMotor.getCurrentPosition() / ticksPerRadian;
        // was not anglewrapped before
        // TODO: new idea (and retune pids: convert new angle to ticks and set it)
//        double c = turretMotor.getCurrentPosition();
//        double t = target * ticksPerRadian;
//        double power = turretController.calculate(c, t);
        double power = turretController.calculate(current, target);
        turretMotor.setPower(power);
    }

    @Override
    public void start() {

    }

    public void setTarget(double x) {
        target = x;
    }


}
