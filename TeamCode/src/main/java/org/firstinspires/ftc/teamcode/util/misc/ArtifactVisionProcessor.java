package org.firstinspires.ftc.teamcode.util.misc;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public abstract class ArtifactVisionProcessor implements VisionProcessor {

    // ===== HSV thresholds =====
    private static final Scalar LOWER_GREEN = new Scalar(35, 30, 30);
    private static final Scalar UPPER_GREEN = new Scalar(90, 255, 255);

    private static final Scalar LOWER_PURPLE = new Scalar(125, 50, 50);
    private static final Scalar UPPER_PURPLE = new Scalar(155, 255, 255);

    // ===== Homography (ASSUMED GIVEN) =====
    // 3x3 homography matrix
    private final Mat H;

    // ===== Output =====
    private final Point worldPoint = new Point(0, 0);

    public ArtifactVisionProcessor(double[][] homography) {
        H = new Mat(3, 3, CvType.CV_64F);
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                H.put(r, c, homography[r][c]);
            }
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Nothing needed
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.medianBlur(hsv, hsv, 5);

        // Masks
        Mat maskGreen = new Mat();
        Mat maskPurple = new Mat();
        Mat combinedMask = new Mat();

        Core.inRange(hsv, LOWER_GREEN, UPPER_GREEN, maskGreen);
        Core.inRange(hsv, LOWER_PURPLE, UPPER_PURPLE, maskPurple);
        Core.bitwise_or(maskGreen, maskPurple, combinedMask);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(
                combinedMask,
                contours,
                new Mat(),
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        if (contours.isEmpty()) {
            worldPoint.x = 0;
            worldPoint.y = 0;
            return null;
        }

        // Largest contour
        MatOfPoint largest = contours.get(0);
        double maxArea = Imgproc.contourArea(largest);

        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area > maxArea) {
                maxArea = area;
                largest = c;
            }
        }

        // Centroid
        Moments m = Imgproc.moments(largest);
        if (m.m00 == 0) {
            worldPoint.x = 0;
            worldPoint.y = 0;
            return null;
        }

        double cx = m.m10 / m.m00;
        double cy = m.m01 / m.m00;

        // Image → World
        worldPoint.x = imageToWorld(cx, cy).x;
        worldPoint.y = imageToWorld(cx, cy).y;

        // Optional visualization
        Imgproc.circle(frame, new Point(cx, cy), 8, new Scalar(0, 255, 0), 2);

        return null;
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

    public Point getWorldPoint() {
        return worldPoint;
    }
}

