package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.misc.ArtifactProcessor;
import org.firstinspires.ftc.teamcode.util.misc.ArtifactVisionProcessor;
import org.firstinspires.ftc.teamcode.util.misc.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;


public class ArtifactVision extends Subsystem {
    private VisionPortal portal;
    private ArtifactProcessor colorLocator;
    private double bestX = -19;
    private Mat H;
    public boolean colorLocatorNull = true;
    public boolean hasMaxArea = false;
    private double pathTime = 0.8; // path usually takes 0.8s i guess
    private double g = 9.8;
    private double h = 0.61;
    private double uk = 0.55;
    private double energyScaleFactor = 0.2; // idk what im doing but compensates for friction energy lost on ramp + air resistance + if ball hits gate and weird stuff, this factor makes sense physics-wise
    private double uk2 = 0.25; // random estimate lol
    private ElapsedTime elapsedTime;
    private double prevX = 0;
    private double currentV = 0;
    int frames = 0;
    // TODO: make it not scan the top half or eliminate anything in the top half
    // change roi in the vision processor impl or something

    public ArtifactVision(HardwareMap hardwareMap) {
        colorLocator = new ArtifactProcessor.Builder()
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        double[][] homography = {
            { -4.98885709e-02, 2.91967072e-02, 9.88258249e+00 },
            { 3.80623143e-03,  5.18877858e-02, -2.55240985e+01 },
            { -6.01290894e-05, -4.77537046e-03,  1.00000000e+00 }
        };

        // TODO: different homography matrix for red
        this.H = new Mat(3, 3, CvType.CV_64F);
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                H.put(r, c, homography[r][c]);
            }
        }

    }

    @Override
    public void update() {
        frames++;
        if (colorLocator == null) {
            colorLocatorNull = true;
            return;
        } else {
            colorLocatorNull = false;
        }

        List<ArtifactProcessor.Blob> blobs = colorLocator.getBlobs();
        if (blobs == null) {
            return;
        }

        ArtifactProcessor.Util.filterByCriteria(
                ArtifactProcessor.BlobCriteria.BY_CONTOUR_AREA,
                300, 20000, blobs);  // filter out very small blobs.

        if (blobs.isEmpty()) { return; }

        // new idea method: find world x and y, loop through a range, range with most total area wins
        // may want to subtract a bit, because balls usually have a bit of downward velocity

        List<Double> distances = new ArrayList<>();
        List<Double> areas = new ArrayList<>(); // calculating these first to save on computations
        for(ArtifactProcessor.Blob b : blobs)
        {
            RotatedRect boxFit = b.getBoxFit();
            distances.add(imageToWorld(boxFit.center.x, boxFit.center.y).x);
            areas.add((double) b.getContourArea());
        }

        double maxAreaLoc = -16;
        double maxArea = 0;

        hasMaxArea = false;
        for (int i = -240; i < 300; i++) {
            double area = calculateArea(distances, areas, i/10d-5, i/10d+5);
            hasMaxArea = true;
            if (area > maxArea) {
                maxArea = area;
                maxAreaLoc = i/10d;
            }
        }
        if (frames % 10 == 0) { // check velocity every 10 frames idk why i chose 10
            double dt = elapsedTime.seconds();
            if (dt > 0) { // no div 0 errors pls
                currentV = (maxAreaLoc - prevX) / dt;
                if (currentV > 0) {currentV = 0;} // balls should NOT be going right, if so it's a mistake
                prevX = maxAreaLoc;
                elapsedTime.reset();
            }
        }
        // x = vot + 1/2 at^2, but change to meters first, then back to inches
        // assuming v <= 0 then friction acts in the opposite direction
        // time when v = 0.
        double offsetX = 0;
        double timeAtStop = (currentV / 39.37) / (uk2 * g);
        if (timeAtStop < pathTime) {
            // if the ball stops early, stop extrapolating its position
            // from the work-kinetic energy thm:
            offsetX = (Math.pow((currentV / 39.37), 2) / (2 * uk2 * g)) * 39.37;
        } else {
            // the ball is still moving before the robot finishes the path.
            // from basic kinematics:
            offsetX = ((currentV/39.37) * pathTime + 0.5 * (g * uk2) * pathTime * pathTime);
        }
        if (offsetX > 0) {offsetX = 0;} // offset should be negative
        // postprocessing: compensating for ball velocity
        // double offsetX = calculateVelocity(maxAreaLoc) * pathTime;
        // better postprocessing
        maxAreaLoc -= offsetX;
        // making sure it doesn't aim too low!
        if (maxAreaLoc < -16) { maxAreaLoc = -16; }

        bestX = maxAreaLoc;
    }

    @Override
    public void start() {
        elapsedTime.reset();
    }

    private double calculateVelocity(double x) {
        // step 1: -24 gets mapped to 0, 20 gets mapped to 44, and we do 72-xloc to find d
        double d = (72 - (x + 24)) / 39.37;
        if (h-uk*d < 0) { return 0; }
        return Math.sqrt(((6/5d)*g*energyScaleFactor) * (h-uk*d)) * 39.37; // back to inches
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
    private double calculateArea(List<Double> dists, List<Double> areas, double lower, double upper) {
                double totalArea = 0;
                for (int i = 0; i < dists.size(); i++) {
                    double x = dists.get(i);
                    if (x >= lower && x <= upper) {
                        totalArea += areas.get(i);
                    }
                }
                return totalArea;
    }
}
