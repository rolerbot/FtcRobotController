package org.firstinspires.ftc.teamcode.opencv_pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class AstraHoughPipeline extends OpenCvPipeline {

    // --- HOUGH PARAMETERS (Tune these in EOCV-Sim) ---
    public static double DP = 1.2;          // Resolution of the accumulator
    public static double MIN_DIST = 100;    // Min distance between centers of circles
    public static double CANNY_THR = 100;   // Higher threshold for Canny edge detector
    public static double ACCUM_THR = 30;    // Lower = more circles (even faint ones)
    public static int MIN_RAD = 30;
    public static int MAX_RAD = 300;

    // --- COLOR THRESHOLDS ---
    public static Scalar PURPLE_LOW = new Scalar(120, 60, 50);
    public static Scalar PURPLE_HIGH = new Scalar(165, 255, 255);
    public static Scalar GREEN_LOW = new Scalar(35, 60, 50);
    public static Scalar GREEN_HIGH = new Scalar(95, 255, 255);

    private final Mat gray = new Mat();
    private final Mat hsv = new Mat();
    private final Mat circles = new Mat();

    public static class Ball {
        public String color;
        public Point center;
        public Ball(String c, Point p) { color = c; center = p; }
    }

    private volatile List<Ball> detectedBalls = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        // 1. Prepare Image (Grayscale + Blur is required for Hough)
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.GaussianBlur(gray, gray, new Size(9, 9), 2, 2);

        // 2. Prepare HSV for color checking
        if (input.channels() == 4) Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGBA2RGB);
        else input.copyTo(hsv);
        Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV);

        // 3. Find Circles
        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, DP, MIN_DIST, CANNY_THR, ACCUM_THR, MIN_RAD, MAX_RAD);

        List<Ball> frameBalls = new ArrayList<>();

        for (int i = 0; i < circles.cols(); i++) {
            double[] data = circles.get(0, i);
            Point center = new Point(Math.round(data[0]), Math.round(data[1]));
            int radius = (int) Math.round(data[2]);

            // 4. Sample the center of the circle to identify color
            String color = getCenterColor(center);

            if (!color.equals("NONE")) {
                frameBalls.add(new Ball(color, center));

                // Visualization
                Scalar drawCol = color.equals("PURPLE") ? new Scalar(255, 0, 255) : new Scalar(0, 255, 0);
                Imgproc.circle(input, center, radius, drawCol, 4); // Draw the circle
                Imgproc.circle(input, center, 5, new Scalar(255, 255, 255), -1); // Draw center dot
                Imgproc.putText(input, color, new Point(center.x - 20, center.y - radius - 10), 0, 0.8, new Scalar(255,255,255), 2);
            }
        }

        detectedBalls = frameBalls;
        return input;
    }

    private String getCenterColor(Point p) {
        if (p.x < 0 || p.y < 0 || p.x >= hsv.cols() || p.y >= hsv.rows()) return "NONE";

        // Sample center pixel HSV
        double[] pixel = hsv.get((int)p.y, (int)p.x);
        Scalar sample = new Scalar(pixel[0], pixel[1], pixel[2]);

        if (checkRange(sample, PURPLE_LOW, PURPLE_HIGH)) return "PURPLE";
        if (checkRange(sample, GREEN_LOW, GREEN_HIGH)) return "GREEN";
        return "NONE";
    }

    private boolean checkRange(Scalar s, Scalar low, Scalar high) {
        return s.val[0] >= low.val[0] && s.val[0] <= high.val[0] &&
                s.val[1] >= low.val[1] && s.val[1] <= high.val[1] &&
                s.val[2] >= low.val[2] && s.val[2] <= high.val[2];
    }

    public List<Ball> getDetectedBalls() { return detectedBalls; }
}