package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.misc.ArtifactProcessor;
import org.firstinspires.ftc.teamcode.util.misc.ArtifactVisionProcessor;
import org.firstinspires.ftc.teamcode.util.misc.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.List;


public class ArtifactVision extends Subsystem {
    private VisionPortal portal;
    private ArtifactProcessor colorLocator;
    private double bestX = -19;
    private Mat H;

    public ArtifactVision(HardwareMap hardwareMap) {
        ArtifactProcessor colorLocator = new ArtifactProcessor.Builder()
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        double[][] homography = {
            { 0.012, -0.0003, -24 },
            { 0.0001, -0.015, 40 },
            { 0.00002, -0.00001, 1 }
        };

        this.H = new Mat(3, 3, CvType.CV_64F);
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                H.put(r, c, homography[r][c]);
            }
        }
    }

    @Override
    public void update() {
        List<ArtifactProcessor.Blob> blobs = colorLocator.getBlobs();

        ArtifactProcessor.Util.filterByCriteria(
                ArtifactProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, blobs);  // filter out very small blobs.

        if (blobs.isEmpty()) { return; }

        // new idea method: find world x and y, loop through a range, range with most total area wins
        // may want to subtract a bit, because balls usually have a bit of downward velocity

        List<Double> distances = new ArrayList<>();
        for(ArtifactProcessor.Blob b : blobs)
        {
            RotatedRect boxFit = b.getBoxFit();
            distances.add(imageToWorld(boxFit.center.x, boxFit.center.y).x);
        }

        double maxAreaLoc = -19;
        double maxArea = 0;

        for (int i = -19; i < 19; i++) {
            double area = calculateArea(distances, blobs, i-5, i+5);
            if (area > maxArea) {
                maxArea = area;
                maxAreaLoc = i;
            }
        }
        bestX = maxAreaLoc;
    }

    @Override
    public void start() {
    }

    public double getLargestClusterX() {
        return bestX;
    }
    private Point imageToWorld(double x, double y) {
        Mat pt = new Mat(3, 1, CvType.CV_64F);
        pt.put(0, 0, x);
        pt.put(1, 0, y);
        pt.put(2, 0, 1.0);

        Mat world = new Mat();
        Core.gemm(H, pt, 1, new Mat(), 0, world);

        double wx = world.get(0, 0)[0];
        double wy = world.get(1, 0)[0];
        double w  = world.get(2, 0)[0];

        if (w == 0) return new Point(0, 0);

        return new Point(wx / w, wy / w);
    }
    private double calculateArea(List<Double> dists, List<ArtifactProcessor.Blob> blobs, double lower, double upper) {
                double totalArea = 0;
                for (int i = 0; i < dists.size(); i++) {
                    double x = dists.get(i);
                    if (x >= lower && x <= upper) {
                        totalArea += blobs.get(i).getContourArea();
                    }
                }
                return totalArea;
    }
}
