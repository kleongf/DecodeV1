package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Point;

@TeleOp(name="vision processor test")
public class VisionTestProcessorReal extends OpMode {
    private  ArtifactProcessorTest vision;

    @Override
    public void loop() {
        Point world = vision.getWorldPosition();
        telemetry.addData("World X", world.x);
        telemetry.addData("World Y", world.y);
        telemetry.update();

        telemetry.update();
    }

    @Override
    public void init() {
        ArtifactProcessorTest vision = new ArtifactProcessorTest(hardwareMap);
    }

    @Override
    public void start() {

    }

    private double normalizeInput(double input) {
        return Math.signum(input) * Math.sqrt(Math.abs(input));
    }
}

