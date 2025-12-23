package org.firstinspires.ftc.teamcode.opencv_pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class SovereignColorPipeline extends OpenCvPipeline {

    // --- DASHBOARD CONSTANTS ---
    public static Scalar PURPLE_LOW = new Scalar(120, 60, 50);
    public static Scalar PURPLE_HIGH = new Scalar(165, 255, 255);
    public static Scalar GREEN_LOW = new Scalar(35, 60, 50);
    public static Scalar GREEN_HIGH = new Scalar(90, 255, 255);

    public static int DEBOUNCE_THRESHOLD = 5; // Frames required to "lock in" a color
    public static double CONFIDENCE_WEIGHT = 0.70; // 70% dominance required
    public static int MIN_SATURATION = 70; // Ignore washed-out gray colors

    // --- INTERNAL STORAGE ---
    private final Mat hsv = new Mat();
    private final Mat thresholdMask = new Mat();
    private final Mat overlay = new Mat();
    private final List<Mat> sectorMasks = new ArrayList<>();

    // --- TEMPORAL STATE (The "Brain") ---
    private final int[][] frameCounters = new int[3][2]; // [Sector][Purple, Green]
    private final String[] confirmedResults = {"NONE", "NONE", "NONE"};

    private boolean initialized = false;
    private Point center;
    private int radius;

    @Override
    public Mat processFrame(Mat input) {
        if (!initialized) initPipeline(input);

        // 1. Dual-Space Filtering
        // Convert to HSV and handle RGBA/RGB automatically
        if (input.channels() == 4) Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGBA2RGB);
        else input.copyTo(hsv);
        Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV);

        overlay.setTo(new Scalar(0, 0, 0, 0));

        for (int i = 0; i < 3; i++) {
            // A. Analyze Purple
            Core.inRange(hsv, PURPLE_LOW, PURPLE_HIGH, thresholdMask);
            double purpleScore = getFilteredScore(i);

            // B. Analyze Green
            Core.inRange(hsv, GREEN_LOW, GREEN_HIGH, thresholdMask);
            double greenScore = getFilteredScore(i);

            // 2. Logic: Temporal Hysteresis (The Stabilizer)
            String frameWinner = "NONE";
            double total = purpleScore + greenScore;

            if (total > 500) { // Noise Floor
                if (purpleScore > total * CONFIDENCE_WEIGHT) frameWinner = "PURPLE";
                else if (greenScore > total * CONFIDENCE_WEIGHT) frameWinner = "GREEN";
            }

            updateDebounce(i, frameWinner);
            renderSector(i);
        }

        // 3. Final Compositing
        Core.addWeighted(input, 1.0, overlay, 0.4, 0, input);
        drawHUD(input);

        return input;
    }

    private double getFilteredScore(int sectorIdx) {
        // bitwise_and the threshold with the specific sector slice
        Core.bitwise_and(thresholdMask, sectorMasks.get(sectorIdx), thresholdMask);
        return Core.countNonZero(thresholdMask);
    }

    private void updateDebounce(int sectorIdx, String winner) {
        // If purple wins this frame, increment purple counter, reset green
        if (winner.equals("PURPLE")) {
            frameCounters[sectorIdx][0]++;
            frameCounters[sectorIdx][1] = 0;
        } else if (winner.equals("GREEN")) {
            frameCounters[sectorIdx][1]++;
            frameCounters[sectorIdx][0] = 0;
        } else {
            frameCounters[sectorIdx][0] = 0;
            frameCounters[sectorIdx][1] = 0;
            confirmedResults[sectorIdx] = "NONE";
        }

        // Only switch the "Confirmed" result if a counter hits the threshold
        if (frameCounters[sectorIdx][0] >= DEBOUNCE_THRESHOLD) confirmedResults[sectorIdx] = "PURPLE";
        if (frameCounters[sectorIdx][1] >= DEBOUNCE_THRESHOLD) confirmedResults[sectorIdx] = "GREEN";
    }

    private void renderSector(int i) {
        Scalar color = new Scalar(60, 60, 60, 255); // Default Gray
        if (confirmedResults[i].equals("PURPLE")) color = new Scalar(200, 0, 255, 255);
        if (confirmedResults[i].equals("GREEN")) color = new Scalar(0, 255, 100, 255);

        Imgproc.ellipse(overlay, center, new Size(radius, radius), 0,
                i * 120, (i * 120) + 120, color, -1);
    }

    private void initPipeline(Mat input) {
        center = new Point(input.cols() / 2.0, input.rows() / 2.0);
        radius = Math.min(input.cols(), input.rows()) / 2;
        overlay.create(input.size(), input.type());

        for (int i = 0; i < 3; i++) {
            Mat m = new Mat(input.size(), CvType.CV_8UC1, new Scalar(0));
            Imgproc.ellipse(m, center, new Size(radius, radius), 0, i * 120, (i * 120) + 120, new Scalar(255), -1);
            sectorMasks.add(m);
        }
        initialized = true;
    }

    private void drawHUD(Mat input) {
        Imgproc.circle(input, center, radius, new Scalar(255, 255, 255), 2);
        for (int i = 0; i < 3; i++) {
            double angle = Math.toRadians(i * 120);
            Point p2 = new Point(center.x + radius * Math.cos(angle), center.y + radius * Math.sin(angle));
            Imgproc.line(input, center, p2, new Scalar(255, 255, 255), 2);

            // Draw result text inside each slice
            double textAngle = Math.toRadians(i * 120 + 60);
            Point textPos = new Point(center.x + (radius * 0.6) * Math.cos(textAngle), center.y + (radius * 0.6) * Math.sin(textAngle));
            Imgproc.putText(input, confirmedResults[i], textPos, 0, 0.5, new Scalar(255, 255, 255), 2);
        }
    }

    public String getConfirmedResult(int i) { return confirmedResults[i]; }
}