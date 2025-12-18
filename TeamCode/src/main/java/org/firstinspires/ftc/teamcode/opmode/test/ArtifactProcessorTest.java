package org.firstinspires.ftc.teamcode.opmode.test;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.misc.ArtifactVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArtifactProcessorTest {

    private final VisionPortal portal;
    private final ArtifactVisionProcessor processor;

    public ArtifactProcessorTest(HardwareMap hardwareMap) {

        // Example homography (REPLACE WITH YOUR REAL ONE)
        double[][] H = {
                { 0.012, -0.0003, -24 },
                { 0.0001, -0.015, 40 },
                { 0.00002, -0.00001, 1 }
        };

        processor = new ArtifactVisionProcessor(H) {
            @Override
            public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

            }
        };

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processor)
                .build();
    }

    /** Returns world (x, y). If none detected → (0, 0). */
    public Point getWorldPosition() {
        return processor.getWorldPoint();
    }

    public void stop() {
        portal.close();
    }
}

