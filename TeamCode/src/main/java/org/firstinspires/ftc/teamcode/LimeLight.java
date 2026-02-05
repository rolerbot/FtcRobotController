package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.PinpointBlocksDriver.GoBildaPinpointDriver;

public class LimeLight
{
    private Limelight3A limelight;
    private int IdTag = 0;
    private boolean IsBlue = true;
    private GoBildaPinpointDriver pinpoint;

    public LimeLight(boolean IsBlue)
    {
        this.IsBlue = IsBlue;
    }

    private void LinkComponents(HardwareMap hardwareMap)
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public void Initialize(HardwareMap hardwareMap)
    {
        LinkComponents(hardwareMap);
        pinpoint.setOffsets(2.11, -3.31);
        limelight.start();
    }

    public void Run()
    {
        if(IsBlue)
            RelocalizationBlue();
        else
            RelocalizationRed();
    }

    public Limelight3A getLimelight() {return limelight;}

    public void RelocalizationRed()
    {
        // Switch to pipeline for red tag
        limelight.pipelineSwitch(0); // Set to the number you assign Red Tag in the Limelight UI
        Relocalization();
    }
    public void RelocalizationBlue()
    {
        // Switch to pipeline for blue tag
        limelight.pipelineSwitch(1); // Set to the number you assign Blue Tag in the Limelight UI
        Relocalization();
    }
    private void Relocalization()
    {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid() && llResult.getFiducialResults() != null && !llResult.getFiducialResults().isEmpty())
        {
            // Get the robot's position on the field as calculated by Limelight
            Pose3D robotPoseField = llResult.getFiducialResults().get(0).getRobotPoseFieldSpace();

            if (robotPoseField != null)
            {
                double robotX_m = robotPoseField.getPosition().x; // meters
                double robotY_m = robotPoseField.getPosition().y; // meters
                double robotYaw_deg = robotPoseField.getOrientation().getYaw(AngleUnit.DEGREES); // degrees

                // Convert to inches for Pinpoint coordinates (1m = 39.3701in)
                double robotX_in = robotX_m * 39.3701;
                double robotY_in = robotY_m * 39.3701;

                // Set pose in Pinpoint using Pose2D
                Pose2D newPose = new Pose2D(DistanceUnit.INCH, robotX_in, robotY_in, AngleUnit.DEGREES, robotYaw_deg);

                pinpoint.setPosition(newPose);
                pinpoint.recalibrateIMU();
            }
        }
    }
}