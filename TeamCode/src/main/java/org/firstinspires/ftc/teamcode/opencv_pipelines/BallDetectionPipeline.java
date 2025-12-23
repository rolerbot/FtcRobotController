package org.firstinspires.ftc.teamcode.opencv_pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class BallDetectionPipeline extends OpenCvPipeline {

    public static Scalar PURPLE_LOW = new Scalar(120, 60, 50);
    public static Scalar PURPLE_HIGH = new Scalar(165, 255, 255);
    public static Scalar GREEN_LOW = new Scalar(35, 60, 50);
    public static Scalar GREEN_HIGH = new Scalar(90, 255, 255);
    public static double MIN_AREA = 1000;
    public static double MIN_CIRCULARITY = 0.6; // To verify objects are ball-shaped
    public static double MAX_AREA = 50000; // Maximum expected area for a single ball
    public static boolean SHOW_DEBUG = true; // Toggle debug visualizations

    // A simple class to hold what we found
    public static class DetectedObject {
        public String color;
        public Point center;
        public double area;
        public double radius;
        public DetectedObject(String c, Point p, double a, double r) {
            color = c; center = p; area = a; radius = r;
        }
    }

    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat distTransform = new Mat();
    private Mat markers = new Mat();
    private final Mat sure_fg = new Mat();
    private final Mat unknown = new Mat();
    private final Mat sure_bg = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));

    // This list is what your OpMode will read
    private volatile List<DetectedObject> latestDetections = new ArrayList<>();

    // Store debug info for drawing
    private static class DebugInfo {
        List<MatOfPoint> overlappedContours = new ArrayList<>();
        List<Point> detectedCenters = new ArrayList<>();
        List<Point> watershedBoundaries = new ArrayList<>();
        List<MatOfPoint> separatedContours = new ArrayList<>();

        void clear() {
            for (MatOfPoint m : overlappedContours) m.release();
            for (MatOfPoint m : separatedContours) m.release();
            overlappedContours.clear();
            detectedCenters.clear();
            watershedBoundaries.clear();
            separatedContours.clear();
        }
    }
    private DebugInfo debugInfo = new DebugInfo();

    @Override
    public Mat processFrame(Mat input) {
        // Clear previous debug info
        if (SHOW_DEBUG) {
            debugInfo.clear();
        }

        // 1. Convert to HSV (Auto-handle RGBA/RGB)
        if (input.channels() == 4) Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGBA2RGB);
        else input.copyTo(hsv);
        Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV);

        List<DetectedObject> currentFrameObjects = new ArrayList<>();

        // 2. Find Purple and Green Blobs with overlap handling
        findColorWithSeparation(currentFrameObjects, PURPLE_LOW, PURPLE_HIGH, "PURPLE");
        findColorWithSeparation(currentFrameObjects, GREEN_LOW, GREEN_HIGH, "GREEN");

        // 3. Sort by Area (Biggest objects first)
        currentFrameObjects.sort((o1, o2) -> Double.compare(o2.area, o1.area));

        // 4. Limit to max 3 balls (as specified in requirements)
        if (currentFrameObjects.size() > 3) {
            currentFrameObjects = currentFrameObjects.subList(0, 3);
        }

        // 5. Update the volatile list for the OpMode
        latestDetections = currentFrameObjects;

        // 6. Draw debug visualizations if enabled
        if (SHOW_DEBUG) {
            // Draw overlapped contours in yellow
            for (MatOfPoint contour : debugInfo.overlappedContours) {
                Imgproc.drawContours(input, List.of(contour), -1, new Scalar(0, 255, 255), 2);

                // Add "OVERLAPPED" label
                Moments m = Imgproc.moments(contour);
                if (m.m00 != 0) {
                    Point center = new Point(m.m10 / m.m00, m.m01 / m.m00);
                    Imgproc.putText(input, "OVERLAPPED", new Point(center.x - 40, center.y - 30),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 255), 2);
                }
            }

            // Draw detected centers (cyan circles with yellow rings)
            for (Point center : debugInfo.detectedCenters) {
                Imgproc.circle(input, center, 8, new Scalar(255, 255, 0), -1); // Cyan center
                Imgproc.circle(input, center, 12, new Scalar(0, 255, 255), 2); // Yellow ring
            }

            // Draw watershed boundaries in red
            for (Point boundary : debugInfo.watershedBoundaries) {
                Imgproc.circle(input, boundary, 1, new Scalar(0, 0, 255), -1);
            }

            // Draw separated contours in bright green
            for (MatOfPoint contour : debugInfo.separatedContours) {
                Imgproc.drawContours(input, List.of(contour), -1, new Scalar(0, 255, 0), 2);
            }

            // Draw debug legend
            int yPos = 20;
            Imgproc.putText(input, "DEBUG MODE", new Point(10, yPos),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 0), 2);
            yPos += 25;
            Imgproc.putText(input, "Yellow = Overlapped", new Point(10, yPos),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(0, 255, 255), 1);
            yPos += 20;
            Imgproc.putText(input, "Cyan/Yellow = Centers", new Point(10, yPos),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(255, 255, 0), 1);
            yPos += 20;
            Imgproc.putText(input, "Red = Boundaries", new Point(10, yPos),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(0, 0, 255), 1);
            yPos += 20;
            Imgproc.putText(input, "Green = Separated", new Point(10, yPos),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(0, 255, 0), 1);
        }

        // 7. Draw final detected balls (always shown)
        for (DetectedObject obj : currentFrameObjects) {
            Scalar color = obj.color.equals("PURPLE") ? new Scalar(255, 0, 255) : new Scalar(0, 255, 0);
            Imgproc.circle(input, obj.center, (int)obj.radius, color, 2);
            Imgproc.circle(input, obj.center, 5, color, -1);
            Imgproc.putText(input, obj.color + " (" + (int)obj.center.x + "," + (int)obj.center.y + ")",
                    new Point(obj.center.x + 15, obj.center.y - 15),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
        }

        return input;
    }

    private void findColorWithSeparation(List<DetectedObject> list, Scalar low, Scalar high,
                                         String name) {
        // Create mask for this color
        Core.inRange(hsv, low, high, mask);

        // Morphological operations to clean up noise
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        // Find initial contours to check if we need separation
        List<MatOfPoint> initialContours = new ArrayList<>();
        Imgproc.findContours(mask.clone(), initialContours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Check each contour - if too large or not circular enough, apply watershed
        for (MatOfPoint contour : initialContours) {
            double area = Imgproc.contourArea(contour);

            if (area < MIN_AREA) {
                contour.release();
                continue;
            }

            // Calculate circularity: 4π*area / perimeter²
            double perimeter = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
            double circularity = (4 * Math.PI * area) / (perimeter * perimeter);

            // If contour is suspiciously large or not circular, it might be overlapping balls
            if (area > MAX_AREA || circularity < MIN_CIRCULARITY) {
                // Store for debug drawing
                if (SHOW_DEBUG) {
                    MatOfPoint clonedContour = new MatOfPoint();
                    contour.copyTo(clonedContour);
                    debugInfo.overlappedContours.add(clonedContour);
                }

                // Use watershed to separate
                separateOverlappingBalls(mask, contour, list, name);
            } else {
                // Single ball detected - add it directly
                Moments m = Imgproc.moments(contour);
                if (m.m00 != 0) {
                    Point center = new Point(m.m10 / m.m00, m.m01 / m.m00);
                    double radius = Math.sqrt(area / Math.PI);
                    list.add(new DetectedObject(name, center, area, radius));
                }
            }
            contour.release();
        }
    }

    private void separateOverlappingBalls(Mat colorMask, MatOfPoint suspiciousContour,
                                          List<DetectedObject> list, String name) {
        // Create a mask for just this contour
        Mat singleObjectMask = Mat.zeros(colorMask.size(), CvType.CV_8UC1);
        Imgproc.drawContours(singleObjectMask, List.of(suspiciousContour), -1,
                new Scalar(255), -1);

        // Distance transform - finds the center regions of each ball
        Imgproc.distanceTransform(singleObjectMask, distTransform, Imgproc.DIST_L2, 5);
        Core.normalize(distTransform, distTransform, 0, 1.0, Core.NORM_MINMAX);

        // Threshold to get sure foreground (ball centers)
        // Adjust threshold (0.4-0.6) based on how much overlap you expect
        Imgproc.threshold(distTransform, sure_fg, 0.5, 1.0, Imgproc.THRESH_BINARY);
        sure_fg.convertTo(sure_fg, CvType.CV_8UC1);

        // Sure background area - dilate the mask
        Imgproc.dilate(singleObjectMask, sure_bg, kernel, new Point(-1, -1), 3);

        // Unknown region - area between foreground and background
        Core.subtract(sure_bg, sure_fg, unknown);

        // Label markers for watershed
        List<MatOfPoint> fgContours = new ArrayList<>();
        Imgproc.findContours(sure_fg.clone(), fgContours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        markers = Mat.zeros(colorMask.size(), CvType.CV_32SC1);

        // Mark each foreground region with a different label
        for (int i = 0; i < fgContours.size(); i++) {
            Imgproc.drawContours(markers, fgContours, i, new Scalar(i + 1), -1);

            // Store detected centers for debug drawing
            if (SHOW_DEBUG) {
                Moments m = Imgproc.moments(fgContours.get(i));
                if (m.m00 != 0) {
                    Point center = new Point(m.m10 / m.m00, m.m01 / m.m00);
                    debugInfo.detectedCenters.add(center);
                }
            }

            fgContours.get(i).release();
        }

        // Mark unknown regions as 0
        markers.setTo(new Scalar(0), unknown);

        // Prepare 3-channel image for watershed
        Mat rgb = new Mat();
        Imgproc.cvtColor(colorMask, rgb, Imgproc.COLOR_GRAY2BGR);

        // Apply watershed
        Imgproc.watershed(rgb, markers);

        // Store watershed boundaries for debug drawing
        if (SHOW_DEBUG) {
            for (int i = 0; i < markers.rows(); i++) {
                for (int j = 0; j < markers.cols(); j++) {
                    int label = (int) markers.get(i, j)[0];
                    // Watershed boundaries are marked as -1
                    if (label == -1) {
                        debugInfo.watershedBoundaries.add(new Point(j, i));
                    }
                }
            }
        }

        // Extract separated balls
        for (int label = 1; label <= fgContours.size(); label++) {
            Mat labelMask = new Mat();
            Core.compare(markers, new Scalar(label), labelMask, Core.CMP_EQ);
            labelMask.convertTo(labelMask, CvType.CV_8UC1);

            // Find contour of this separated region
            List<MatOfPoint> separatedContours = new ArrayList<>();
            Imgproc.findContours(labelMask.clone(), separatedContours, new Mat(),
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : separatedContours) {
                double area = Imgproc.contourArea(contour);
                if (area > MIN_AREA) {
                    Moments m = Imgproc.moments(contour);
                    if (m.m00 != 0) {
                        Point center = new Point(m.m10 / m.m00, m.m01 / m.m00);
                        double radius = Math.sqrt(area / Math.PI);
                        list.add(new DetectedObject(name, center, area, radius));

                        // Store separated contour for debug drawing
                        if (SHOW_DEBUG) {
                            MatOfPoint clonedContour = new MatOfPoint();
                            contour.copyTo(clonedContour);
                            debugInfo.separatedContours.add(clonedContour);
                        }
                    }
                }
                contour.release();
            }
            labelMask.release();
        }

        singleObjectMask.release();
        rgb.release();
    }

    public List<DetectedObject> getDetections() {
        return latestDetections;
    }
}