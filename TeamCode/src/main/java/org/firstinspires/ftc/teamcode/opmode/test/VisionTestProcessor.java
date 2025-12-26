package org.firstinspires.ftc.teamcode.opmode.test;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.misc.ArtifactVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

@TeleOp(name="vision processor test")
public class VisionTestProcessor extends OpMode {
    private VisionPortal portal;
    private ArtifactVisionProcessor processor;

    @Override
    public void loop() {
        Point world = processor.getWorldPoint();
        telemetry.addData("World X", world.x);
        telemetry.addData("World Y", world.y);
        telemetry.update();

        telemetry.update();
    }

    @Override
    public void init() {
        // Example homography (REPLACE WITH YOUR REAL ONE)
        double[][] H = {
                { 0.012, -0.0003, -24 },
                { 0.0001, -0.015, 40 },
                { 0.00002, -0.00001, 1 }
        };

        processor = new ArtifactVisionProcessor(H);
        portal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), processor);

//        portal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(processor)
//                .enableLiveView(true)
//                .build();
    }

    @Override
    public void start() {
    }
}

