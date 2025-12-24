package org.firstinspires.ftc.teamcode.opencv_pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class ShapeAwarePiePipeline extends OpenCvPipeline {

    // --- COLOR THRESHOLDS (HSV) ---
    public Scalar lowerGreen = new Scalar(35, 70, 50);
    public Scalar upperGreen = new Scalar(95, 255, 255);
    public Scalar lowerPurple = new Scalar(130, 30, 50); // Pink-inclusive
    public Scalar upperPurple = new Scalar(175, 255, 255);

    // --- SETTINGS ---
    public double ANGLE_OFFSET = 30.0;
    public double MIN_AREA = 600;
    public double MIN_CIRCULARITY = 0.75; // 1.0 is a perfect circle. 0.7-0.8 is usually safe for FTC.

    public enum Content { EMPTY, GREEN, PURPLE }
    private volatile Content[] sectorStates = {Content.EMPTY, Content.EMPTY, Content.EMPTY};

    Mat hsvMat = new Mat();
    Mat greenMask = new Mat();
    Mat purpleMask = new Mat();
    Mat hierarchy = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Content[] frameStates = {Content.EMPTY, Content.EMPTY, Content.EMPTY};

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lowerGreen, upperGreen, greenMask);
        Core.inRange(hsvMat, lowerPurple, upperPurple, purpleMask);

        // Apply Morphological Opening to clean up the masks
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
        Imgproc.morphologyEx(greenMask, greenMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(purpleMask, purpleMask, Imgproc.MORPH_OPEN, kernel);

        analyzeColorAndShape(greenMask, Content.GREEN, frameStates, input);
        analyzeColorAndShape(purpleMask, Content.PURPLE, frameStates, input);

        sectorStates = frameStates;
        drawUI(input);

        kernel.release();
        return input;
    }

    private void analyzeColorAndShape(Mat mask, Content type, Content[] states, Mat display) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Point centerOfImage = new Point(display.width() / 2.0, display.height() / 2.0);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_AREA) {
                // --- CIRCULARITY CHECK ---
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                double perimeter = Imgproc.arcLength(contour2f, true);
                double circularity = (4 * Math.PI * area) / (perimeter * perimeter);

                // Only proceed if it's "round enough"
                if (circularity > MIN_CIRCULARITY) {
                    Moments m = Imgproc.moments(contour);
                    double cx = m.m10 / m.m00;
                    double cy = m.m01 / m.m00;

                    // Angle math with offset
                    double angle = Math.toDegrees(Math.atan2(cy - centerOfImage.y, cx - centerOfImage.x)) + ANGLE_OFFSET;

                    // Normalize to -180 to 180
                    angle = (angle + 180) % 360;
                    if (angle < 0) angle += 360;
                    angle -= 180;

                    int sectorIdx = (angle >= -180 && angle < -60) ? 0 : (angle < 60 ? 1 : 2);
                    states[sectorIdx] = type;

                    // UI: Draw circle around detected object
                    Scalar color = (type == Content.GREEN) ? new Scalar(0, 255, 0) : new Scalar(255, 0, 255);
                    Imgproc.drawContours(display, List.of(contour), -1, color, 3);
                    Imgproc.putText(display, String.format("Circ: %.2f", circularity), new Point(cx, cy - 10), 0, 0.4, color, 1);
                }
            }
        }
    }

    private void drawUI(Mat input) {
        Point center = new Point(input.width()/2.0, input.height()/2.0);
        for (int a : new int[]{-180, -60, 60}) {
            double rad = Math.toRadians(a - ANGLE_OFFSET);
            Imgproc.line(input, center, new Point(center.x + 400 * Math.cos(rad), center.y + 400 * Math.sin(rad)), new Scalar(255, 255, 255), 1);
        }
        Imgproc.putText(input, "S1:"+sectorStates[0]+" S2:"+sectorStates[1]+" S3:"+sectorStates[2], new Point(10, 25), 0, 0.6, new Scalar(0, 255, 255), 2);
    }

    public Content[] getSectorStates() { return sectorStates; }
}