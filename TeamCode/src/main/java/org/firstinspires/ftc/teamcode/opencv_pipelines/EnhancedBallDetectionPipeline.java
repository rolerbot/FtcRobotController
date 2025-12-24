package org.firstinspires.ftc.teamcode.opencv_pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class EnhancedBallDetectionPipeline extends OpenCvPipeline {

    // --- TUNABLE CONSTANTS (FtcDashboard) ---
    public static Scalar PURPLE_LOW = new Scalar(125, 60, 40); // Moderate Saturation
    public static Scalar PURPLE_HIGH = new Scalar(155, 255, 255);

    // Green: Saturation 60 to reduce noise, but not as strict as 80
    public static Scalar GREEN_LOW = new Scalar(30, 60, 40);
    public static Scalar GREEN_HIGH = new Scalar(120, 255, 255);

    // Minimum pixels in a sector to trigger detection
    // Resolution 320x240. Sector ~25k pixels.
    // Threshold 250 = ~1% coverage.
    public static int DETECTION_THRESHOLD = 250;

    public static boolean SHOW_DEBUG = true;

    // --- PROCESSING CONFIG (Restoring 320x240) ---
    private static final int WIDTH = 320;
    private static final int HEIGHT = 240;
    private static final Point CENTER = new Point(WIDTH / 2.0, HEIGHT / 2.0);
    private static final int RADIUS = 250;

    // --- MATRICES & MASKS ---
    private final Mat downscaled = new Mat();
    private final Mat hsv = new Mat();

    private final Mat purpleMask = new Mat();
    private final Mat greenMask = new Mat();

    private final Mat zone1Mask = new Mat(HEIGHT, WIDTH, CvType.CV_8U);
    private final Mat zone2Mask = new Mat(HEIGHT, WIDTH, CvType.CV_8U);
    private final Mat zone3Mask = new Mat(HEIGHT, WIDTH, CvType.CV_8U);

    private final Mat intersection = new Mat();

    // --- RESULTS ---
    private volatile String[] zoneStatus = new String[]{"NONE", "NONE", "NONE"};

    private boolean isInitialized = false;

    @Override
    public void init(Mat firstFrame) {
        createSectorMask(zone1Mask, 0, 120);
        createSectorMask(zone2Mask, 120, 240);
        createSectorMask(zone3Mask, 240, 360);
        isInitialized = true;
    }

    private void createSectorMask(Mat mask, double startAngle, double endAngle) {
        mask.setTo(new Scalar(0));
        List<Point> points = new ArrayList<>();
        points.add(CENTER);

        for (double angle = startAngle; angle <= endAngle; angle += 10) {
            double radians = Math.toRadians(angle);
            double x = CENTER.x + RADIUS * Math.cos(radians);
            double y = CENTER.y + RADIUS * Math.sin(radians);
            points.add(new Point(x, y));
        }
        points.add(CENTER);

        MatOfPoint poly = new MatOfPoint();
        poly.fromList(points);
        Imgproc.fillPoly(mask, java.util.Collections.singletonList(poly), new Scalar(255));
        poly.release();
    }

    @Override
    public Mat processFrame(Mat input) {
        if (!isInitialized) return input;

        // 1. Downscale
        Imgproc.resize(input, downscaled, new Size(WIDTH, HEIGHT));

        // 2. HSV
        Imgproc.cvtColor(downscaled, hsv, Imgproc.COLOR_RGB2HSV);

        // 3. Generate Colors
        Core.inRange(hsv, PURPLE_LOW, PURPLE_HIGH, purpleMask);
        Core.inRange(hsv, GREEN_LOW, GREEN_HIGH, greenMask);

        String[] currentStatus = new String[3];

        // 4. Check Zones
        currentStatus[0] = checkZone(zone1Mask);
        currentStatus[1] = checkZone(zone2Mask);
        currentStatus[2] = checkZone(zone3Mask);

        zoneStatus = currentStatus;

        // 5. Draw Debug (Robust to resolution)
        if (SHOW_DEBUG) {
            drawDebugOverlay(input);
        }

        return input;
    }

    private String checkZone(Mat zoneMask) {
        Core.bitwise_and(purpleMask, zoneMask, intersection);
        if (Core.countNonZero(intersection) > DETECTION_THRESHOLD) return "PURPLE";

        Core.bitwise_and(greenMask, zoneMask, intersection);
        if (Core.countNonZero(intersection) > DETECTION_THRESHOLD) return "GREEN";

        return "NONE";
    }

    private void drawDebugOverlay(Mat input) {
        // Calculate dynamic center and radius based on ACTUAL input resolution
        Point realCenter = new Point(input.width() / 2.0, input.height() / 2.0);
        double rad = Math.min(input.width(), input.height()) * 0.45; // 45% of screen size

        // Draw Sectors
        drawSectorline(input, realCenter, rad, 0);
        drawSectorline(input, realCenter, rad, 120);
        drawSectorline(input, realCenter, rad, 240);

        // Draw Text
        drawStatusText(input, realCenter, rad * 0.6, 60, zoneStatus[0]);
        drawStatusText(input, realCenter, rad * 0.6, 180, zoneStatus[1]);
        drawStatusText(input, realCenter, rad * 0.6, 300, zoneStatus[2]);
    }

    private void drawSectorline(Mat input, Point center, double radius, double angle) {
        double rad = Math.toRadians(angle);
        Point p2 = new Point(center.x + radius * Math.cos(rad), center.y + radius * Math.sin(rad));
        Imgproc.line(input, center, p2, new Scalar(255, 255, 0), 2);
    }

    private void drawStatusText(Mat input, Point center, double radius, double angle, String text) {
        double rad = Math.toRadians(angle);
        double x = center.x + radius * Math.cos(rad) - 40; // simple centering offset
        double y = center.y + radius * Math.sin(rad);

        Scalar color = new Scalar(200, 200, 200); // White-ish default
        if (text.equals("PURPLE")) color = new Scalar(255, 0, 255);
        else if (text.equals("GREEN")) color = new Scalar(0, 255, 0);

        Imgproc.putText(input, text, new Point(x, y), Imgproc.FONT_HERSHEY_SIMPLEX, 1, color, 3);
    }

    public String[] getZoneStatus() {
        return zoneStatus;
    }
}