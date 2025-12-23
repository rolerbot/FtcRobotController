package org.firstinspires.ftc.teamcode.opencv_pipelines;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class ExamplePipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        // input = camera frame
        // You can modify it or analyze it here
        return input; // return image to display on Driver Station
    }
}
