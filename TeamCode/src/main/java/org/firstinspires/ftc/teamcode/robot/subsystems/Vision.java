package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.misc.ArtifactVisionProcessor;
import org.firstinspires.ftc.teamcode.util.misc.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

public class Vision extends Subsystem {
    private VisionPortal portal;
    private ArtifactVisionProcessor processor;
    private Point world = new Point(0, 0);

    public Vision(HardwareMap hardwareMap) {
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

    @Override
    public void update() {
        world = processor.getWorldPoint();
    }

    @Override
    public void start() {
    }

    public double getLargestClusterX() {
        return world.x;
    }
}
