package org.firstinspires.ftc.teamcode.opencv_pipelines;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

/**
 * ULTRA-FAST Green vs Purple Detection Pipeline with Luminance Awareness
 * Uses RGBA color space with perceptual luminance (ITU-R BT.709) for robust detection
 * Optimized specifically for detecting GREEN, PURPLE, or EMPTY in 3 pie slices
 * Handles varying lighting conditions by incorporating luminance into color decisions
 *
 * QUICK TUNING:
 * - Green not detecting? Lower greenAdvantage threshold (line ~235): from 5 to 3
 * - Purple not detecting? Lower purpleAdvantage threshold (line ~290): from 5 to 3
 * - Too many false positives? Increase colorfulness threshold (lines ~241, ~296): from 0.15 to 0.20
 * - Need more/less confidence? Change CONFIDENCE_THRESHOLD (line ~35): current 0.30 (30%)
 */
public class GreenPurpleDetectionPipeline extends OpenCvPipeline {

    // Zone definitions
    private Point centerPoint;
    private int frameWidth;
    private int frameHeight;

    // Results - only 3 possible values: GREEN, PURPLE, EMPTY
    private volatile SliceColor leftSlice;
    private volatile SliceColor centerSlice;
    private volatile SliceColor rightSlice;

    // Configuration
    private boolean showVisualization = true;
    private static final int SAMPLES_PER_SLICE = 369; // Increased for better accuracy

    // Pre-calculated sample points
    private Point[] leftSamples;
    private Point[] centerSamples;
    private Point[] rightSamples;

    // LUMINANCE-BASED COLOR DETECTION
    // Uses RGBA with perceptual luminance (ITU-R BT.709) for robust detection
    // GREEN: RELAXED - Luminance 20-250, Green dominant by 5 units, Colorful >15%, Hue 70-170°
    // PURPLE: Luminance 25-230, R+B>G by 12%, Colorful >20%, R/B balanced, Hue 270-330°

    // Confidence threshold - % of samples that must agree (LOWERED for easier detection)
    private static final double CONFIDENCE_THRESHOLD = 0.30; // 30% of samples must be same color

    // Visualization colors
    private final Scalar WHITE = new Scalar(255, 255, 255);
    private final Scalar BLACK = new Scalar(0, 0, 0);
    private final Scalar GREEN = new Scalar(0, 255, 0);
    private final Scalar PURPLE = new Scalar(255, 0, 255);
    private final Scalar GRAY = new Scalar(128, 128, 128);

    /**
     * Slice color enum - only 3 possible states
     */
    public enum SliceColor {
        GREEN,
        PURPLE,
        EMPTY;

        @Override
        public String toString() {
            return this.name();
        }
    }

    /**
     * Detection result with confidence
     */
    public static class DetectionResult {
        public SliceColor color;
        public double confidence; // 0.0 to 1.0

        public DetectionResult(SliceColor color, double confidence) {
            this.color = color;
            this.confidence = confidence;
        }

        @Override
        public String toString() {
            return String.format("%s (%.0f%%)", color, confidence * 100);
        }
    }

    @Override
    public void init(Mat firstFrame) {
        super.init(firstFrame);
        updateZones(firstFrame.width(), firstFrame.height());
    }

    @Override
    public Mat processFrame(Mat input) {
        // Update zones if frame size changed
        if (centerPoint == null || frameWidth != input.width()) {
            updateZones(input.width(), input.height());
        }

        // Fast detection
        leftSlice = detectSliceColor(input, leftSamples);
        centerSlice = detectSliceColor(input, centerSamples);
        rightSlice = detectSliceColor(input, rightSamples);

        // Return visualization if enabled
        if (showVisualization) {
            return drawVisualization(input);
        }

        return input;
    }

    /**
     * Update zones and pre-calculate sample points
     */
    private void updateZones(int width, int height) {
        frameWidth = width;
        frameHeight = height;
        centerPoint = new Point(width / 2.0, height / 2.0);

        double cx = centerPoint.x;
        double cy = centerPoint.y;

        // LEFT slice: -60° to 60° (top area)
        leftSamples = generateSamplePoints(cx, cy, width, height, -30, 90);

        // CENTER slice: 60° to 180° (right area)
        centerSamples = generateSamplePoints(cx, cy, width, height, 90, 210);

        // RIGHT slice: 180° to 300° (left-bottom area)
        rightSamples = generateSamplePoints(cx, cy, width, height, 210, 330);
    }

    /**
     * Generate evenly distributed sample points in a pie slice
     */
    private Point[] generateSamplePoints(double cx, double cy, int width, int height,
                                         double startAngle, double endAngle) {
        Point[] samples = new Point[SAMPLES_PER_SLICE];

        double startRad = Math.toRadians(startAngle);
        double endRad = Math.toRadians(endAngle);
        double angleRange = endRad - startRad;

        // Simple evenly distributed sampling
        for (int i = 0; i < SAMPLES_PER_SLICE; i++) {
            // Distribute evenly across angles and radii
            double t = i / (double) (SAMPLES_PER_SLICE - 1);
            double angle = startRad + angleRange * t;

            // Alternate between different radii for better coverage
            double radiusFraction = 0.3 + (0.7 * ((i % 5) / 5.0));
            double maxDist = calculateMaxDistanceToEdge(cx, cy, angle, width, height);
            double dist = maxDist * radiusFraction;

            int x = (int) (cx + dist * Math.cos(angle));
            int y = (int) (cy - dist * Math.sin(angle));

            // Clamp to bounds
            x = Math.max(0, Math.min(width - 1, x));
            y = Math.max(0, Math.min(height - 1, y));

            samples[i] = new Point(x, y);
        }

        return samples;
    }

    /**
     * Calculate max distance to frame edge
     */
    private double calculateMaxDistanceToEdge(double cx, double cy, double angle,
                                              int width, int height) {
        double dx = Math.cos(angle);
        double dy = -Math.sin(angle);

        double tRight = (width - cx) / dx;
        double tLeft = -cx / dx;
        double tTop = -cy / dy;
        double tBottom = (height - cy) / dy;

        double t = Double.MAX_VALUE;
        if (dx > 0 && tRight > 0) t = Math.min(t, tRight);
        if (dx < 0 && tLeft > 0) t = Math.min(t, tLeft);
        if (dy > 0 && tBottom > 0) t = Math.min(t, tBottom);
        if (dy < 0 && tTop > 0) t = Math.min(t, tTop);

        return t;
    }

    /**
     * Detect color in a slice - GREEN, PURPLE, or EMPTY
     * Uses RGBA with luminance awareness
     */
    private SliceColor detectSliceColor(Mat image, Point[] samplePoints) {
        if (samplePoints == null) {
            return SliceColor.EMPTY;
        }

        int greenCount = 0;
        int purpleCount = 0;
        int totalSamples = 0;

        // Check each sample point
        for (Point p : samplePoints) {
            if (p == null) continue; // Skip null points

            int x = (int) p.x;
            int y = (int) p.y;

            if (x >= 0 && x < image.cols() && y >= 0 && y < image.rows()) {
                double[] pixel = image.get(y, x);
                if (pixel != null && pixel.length >= 3) {
                    // OpenCV uses BGR format, but we treat as RGB for detection
                    double b = pixel[0];
                    double g = pixel[1];
                    double r = pixel[2];
                    double a = pixel.length > 3 ? pixel[3] : 255; // Alpha if available

                    // Calculate luminance (perceived brightness)
                    // Using ITU-R BT.709 luma coefficients
                    double luminance = 0.2126 * r + 0.7152 * g + 0.0722 * b;

                    // Check if this pixel is green or purple (with luminance)
                    if (isGreenPixelRGBA(r, g, b, a, luminance)) {
                        greenCount++;
                    } else if (isPurplePixelRGBA(r, g, b, a, luminance)) {
                        purpleCount++;
                    }

                    totalSamples++;
                }
            }
        }

        if (totalSamples == 0) {
            return SliceColor.EMPTY;
        }

        // Calculate confidence
        double greenConfidence = greenCount / (double) totalSamples;
        double purpleConfidence = purpleCount / (double) totalSamples;

        // Determine color based on confidence threshold
        if (greenConfidence >= CONFIDENCE_THRESHOLD && greenConfidence > purpleConfidence) {
            return SliceColor.GREEN;
        } else if (purpleConfidence >= CONFIDENCE_THRESHOLD) {
            return SliceColor.PURPLE;
        } else {
            return SliceColor.EMPTY;
        }
    }

    /**
     * Check if a pixel is GREEN using RGBA with luminance awareness
     * RELAXED thresholds for easier green detection
     */
    private boolean isGreenPixelRGBA(double r, double g, double b, double a, double luminance) {
        // Much more relaxed luminance threshold
        if (luminance < 20 || luminance > 250) {
            return false; // Only reject if extremely dark or bright
        }

        // Green must be the highest channel (even if slightly)
        if (g < r || g < b) {
            return false;
        }

        // RELAXED: Green just needs to be somewhat higher than others
        // Old: required 15% dominance, New: just needs to be higher
        double greenAdvantage = g - Math.max(r, b);
        if (greenAdvantage < 5) { // Green must be at least 5 units higher
            return false;
        }

        // RELAXED: Lower colorfulness requirement
        double maxChannel = Math.max(r, Math.max(g, b));
        double minChannel = Math.min(r, Math.min(g, b));
        double colorfulness = (maxChannel - minChannel) / (maxChannel + 1);

        if (colorfulness < 0.15) { // Reduced from 0.25 to 0.15
            return false;
        }

        // RELAXED: Wider hue range
        double delta = maxChannel - minChannel;
        double h = 0;

        if (delta > 5) { // Only calculate hue if there's enough color difference
            if (maxChannel == g) {
                h = 60 * (((b - r) / delta) + 2);
            } else if (maxChannel == r) {
                h = 60 * (((g - b) / delta) % 6);
            } else {
                h = 60 * (((r - g) / delta) + 4);
            }

            if (h < 0) h += 360;

            // Much wider green hue range: 70-170 degrees (was 80-160)
            return (h >= 70 && h <= 170);
        }

        // If delta is small but green is dominant, accept it
        return true;
    }

    /**
     * Check if a pixel is PURPLE using RGBA with luminance awareness
     * Balanced thresholds to match green sensitivity
     */
    private boolean isPurplePixelRGBA(double r, double g, double b, double a, double luminance) {
        // Relaxed luminance threshold
        if (luminance < 20 || luminance > 240) {
            return false; // Reject if extremely dark or bright
        }

        // For purple: red and blue should be higher than green
        if (g >= r || g >= b) {
            return false;
        }

        // RELAXED: Red and blue just need to be somewhat higher than green
        double purpleAdvantage = (r + b) / 2.0 - g;
        if (purpleAdvantage < 5) { // R+B average must be at least 5 units higher than G
            return false;
        }

        // RELAXED: Lower colorfulness requirement
        double maxChannel = Math.max(r, Math.max(g, b));
        double minChannel = Math.min(r, Math.min(g, b));
        double colorfulness = (maxChannel - minChannel) / (maxChannel + 1);

        if (colorfulness < 0.15) { // Reduced from 0.20 to 0.15
            return false;
        }

        // RELAXED: Purple should have relatively balanced red and blue
        double rbRatio = Math.min(r, b) / (Math.max(r, b) + 1);
        if (rbRatio < 0.50) { // Reduced from 0.60 to 0.50 (more tolerant)
            return false;
        }

        // Purple hue verification
        double delta = maxChannel - minChannel;
        double h = 0;

        if (delta > 5) { // Only calculate hue if enough color difference
            if (maxChannel == r) {
                h = 60 * (((g - b) / delta) % 6);
            } else if (maxChannel == b) {
                h = 60 * (((r - g) / delta) + 4);
            } else {
                h = 60 * (((b - r) / delta) + 2);
            }

            if (h < 0) h += 360;

            // Wider purple/magenta hue range: 265-335 degrees (was 270-330)
            return (h >= 265 && h <= 335);
        }

        // If delta is small but purple-ness criteria met, accept it
        return true;
    }

    /**
     * Get detailed detection result with confidence
     */
    public DetectionResult getLeftSliceDetailed() {
        return getDetailedResult(leftSlice, leftSamples);
    }

    public DetectionResult getCenterSliceDetailed() {
        return getDetailedResult(centerSlice, centerSamples);
    }

    public DetectionResult getRightSliceDetailed() {
        return getDetailedResult(rightSlice, rightSamples);
    }

    private DetectionResult getDetailedResult(SliceColor color, Point[] samples) {
        // This would require re-sampling, so for now return with threshold confidence
        double confidence = color == SliceColor.EMPTY ? 0.0 : CONFIDENCE_THRESHOLD;
        return new DetectionResult(color, confidence);
    }

    /**
     * Lightweight visualization
     */
    private Mat drawVisualization(Mat input) {
        Mat output = input.clone();

        int cx = (int) centerPoint.x;
        int cy = (int) centerPoint.y;

        // Draw slice boundaries with color-coding
        Scalar leftColor = getColorForSlice(leftSlice);
        Scalar centerColor = getColorForSlice(centerSlice);
        Scalar rightColor = getColorForSlice(rightSlice);

        drawSliceBoundary(output, cx, cy, -30, leftColor);
        drawSliceBoundary(output, cx, cy, 90, centerColor);
        drawSliceBoundary(output, cx, cy, 210, rightColor);
        drawSliceBoundary(output, cx, cy, 330, leftColor);

        // Draw center point
        Imgproc.circle(output, centerPoint, 5, WHITE, -1);
        Imgproc.circle(output, centerPoint, 5, BLACK, 1);

        // Draw labels with color backgrounds
        drawColorLabel(output, "RIGHT: " + leftSlice, 10, 30, leftColor);
        drawColorLabel(output, "LEFT: " + centerSlice, 10, 60, centerColor);
        drawColorLabel(output, "DOWN: " + rightSlice, 10, 90, rightColor);

        return output;
    }

    /**
     * Get visualization color for a slice
     */
    private Scalar getColorForSlice(SliceColor color) {
        switch (color) {
            case GREEN: return GREEN;
            case PURPLE: return PURPLE;
            case EMPTY: return GRAY;
            default: return WHITE;
        }
    }

    /**
     * Draw slice boundary line
     */
    private void drawSliceBoundary(Mat output, int cx, int cy, double angleDeg, Scalar color) {
        double angleRad = Math.toRadians(angleDeg);
        double maxDist = calculateMaxDistanceToEdge(cx, cy, angleRad, frameWidth, frameHeight);

        int x = (int) (cx + maxDist * Math.cos(angleRad));
        int y = (int) (cy - maxDist * Math.sin(angleRad));

        Imgproc.line(output, centerPoint, new Point(x, y), color, 3);
    }

    /**
     * Draw label with colored background
     */
    private void drawColorLabel(Mat output, String text, int x, int y, Scalar bgColor) {
        // Draw background rectangle
        int textWidth = text.length() * 12;
        Rect bgRect = new Rect(x - 2, y - 18, textWidth, 22);
        Imgproc.rectangle(output, bgRect, bgColor, -1);
        Imgproc.rectangle(output, bgRect, BLACK, 1);

        // Draw text
        Imgproc.putText(output, text, new Point(x, y),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, BLACK, 3);
        Imgproc.putText(output, text, new Point(x, y),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 1);
    }

    // Simple Getters

    public SliceColor getLeftSlice() {
        return leftSlice;
    }

    public SliceColor getCenterSlice() {
        return centerSlice;
    }

    public SliceColor getRightSlice() {
        return rightSlice;
    }

    public Point getCenterPoint() {
        return centerPoint;
    }

    // Configuration

    public void setShowVisualization(boolean show) {
        this.showVisualization = show;
    }

    /**
     * Get compact results string
     */
    public String getResultsString() {
        return String.format("L:%s C:%s R:%s", leftSlice, centerSlice, rightSlice);
    }

    /**
     * Check if a slice has a specific color
     */
    public boolean isColorInSlice(String sliceName, SliceColor targetColor) {
        SliceColor color = null;
        switch (sliceName.toUpperCase()) {
            case "LEFT":
            case "L":
                color = leftSlice;
                break;
            case "CENTER":
            case "C":
            case "MIDDLE":
                color = centerSlice;
                break;
            case "RIGHT":
            case "R":
                color = rightSlice;
                break;
        }

        return color == targetColor;
    }

    /**
     * Find which slice (if any) contains the target color
     */
    public String findSliceWithColor(SliceColor targetColor) {
        if (leftSlice == targetColor) return "LEFT";
        if (centerSlice == targetColor) return "CENTER";
        if (rightSlice == targetColor) return "RIGHT";
        return "NONE";
    }
}