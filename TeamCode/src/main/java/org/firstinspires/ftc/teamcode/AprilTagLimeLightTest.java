package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Limelight Diagnostic")
public class AprilTagLimeLightTest extends OpMode
{
    private Limelight3A limelight;
    private Follower follower;

    @Override
    public void init()
    {
        // Initialize Limelight camera
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        // Initialize Pedro Pathing follower (this includes Pinpoint)
        follower = Constants.createFollower(hardwareMap);

        // Set starting pose (adjust for your field position)
        // Blue alliance example: bottom right corner
        follower.setStartingPose(new Pose(134.6, 9.0, Math.toRadians(90)));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("", "Press START when ready");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        // Update Pedro Pathing follower (updates Pinpoint odometry)
        follower.update();

        // Get current robot pose from Pedro Pathing
        Pose robotPose = follower.getPose();
        double robotX = robotPose.getX();  // inches
        double robotY = robotPose.getY();  // inches
        double robotHeading = Math.toDegrees(robotPose.getHeading());  // degrees

        // Update Limelight with robot orientation
        limelight.updateRobotOrientation(robotHeading);

        // Get Limelight results
        LLResult llResult = limelight.getLatestResult();

        if(llResult != null && llResult.isValid())
        {
            // === GET APRILTAG DETECTION DATA ===
            double tagID = -1;
            int numTagsDetected = 0;
            if (llResult.getFiducialResults() != null && !llResult.getFiducialResults().isEmpty()) {
                numTagsDetected = llResult.getFiducialResults().size();
                tagID = llResult.getFiducialResults().get(0).getFiducialId();
            }

            // Get target tracking data
            double tx = llResult.getTx();  // horizontal offset
            double ty = llResult.getTy();  // vertical offset
            double ta = llResult.getTa();  // target area

            // === GET LIMELIGHT ROBOT POSITION (FTC Coordinates) ===
            Pose3D botPose_FTC = llResult.getBotpose_MT2();  // MegaTag2 uses multiple tags
            double ftcX_m = botPose_FTC.getPosition().x;  // meters
            double ftcY_m = botPose_FTC.getPosition().y;  // meters
            double ftcHeading_deg = botPose_FTC.getOrientation().getYaw(AngleUnit.DEGREES);

            // Convert FTC coordinates (meters) to inches
            double ftcX_inches = ftcX_m * 39.3701;
            double ftcY_inches = ftcY_m * 39.3701;

            // === CONVERT FTC COORDINATES TO PEDRO COORDINATES ===
            Pose limelightPose_FTC = new Pose(ftcX_inches, ftcY_inches, Math.toRadians(ftcHeading_deg), FTCCoordinates.INSTANCE);
            Pose limelightPose_Pedro = limelightPose_FTC.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            // === DISPLAY TELEMETRY ===
            telemetry.addData("═══ APRILTAG DETECTION ═══", "");
            telemetry.addData("Tag ID", tagID != -1 ? String.format("%.0f", tagID) : "NONE");
            telemetry.addData("Num Tags", numTagsDetected);
            telemetry.addData("TX (horiz offset)", tx);
            telemetry.addData("TY (vert offset)", ty);
            telemetry.addData("TA (area)", ta);

            telemetry.addData("", "");
            telemetry.addData("═══ LIMELIGHT POSITION ═══", "");
            telemetry.addData("FTC X", String.format("%.1f\" (%.2fm)", ftcX_inches, ftcX_m));
            telemetry.addData("FTC Y", String.format("%.1f\" (%.2fm)", ftcY_inches, ftcY_m));
            telemetry.addData("FTC Heading", String.format("%.1f°", ftcHeading_deg));

            telemetry.addData("", "");
            telemetry.addData("Pedro X", String.format("%.1f\"", limelightPose_Pedro.getX()));
            telemetry.addData("Pedro Y", String.format("%.1f\"", limelightPose_Pedro.getY()));
            telemetry.addData("Pedro Heading", String.format("%.1f°", Math.toDegrees(limelightPose_Pedro.getHeading())));

            telemetry.addData("", "");
            telemetry.addData("═══ ROBOT POSITION (Odometry) ═══", "");
            telemetry.addData("Pedro X", String.format("%.1f\"", robotX));
            telemetry.addData("Pedro Y", String.format("%.1f\"", robotY));
            telemetry.addData("Pedro Heading", String.format("%.1f°", robotHeading));

            telemetry.addData("", "");
            telemetry.addData("═══ POSITION DIFFERENCE ═══", "");
            double deltaX = Math.abs(limelightPose_Pedro.getX() - robotX);
            double deltaY = Math.abs(limelightPose_Pedro.getY() - robotY);
            double deltaHeading = Math.abs(Math.toDegrees(limelightPose_Pedro.getHeading()) - robotHeading);
            telemetry.addData("ΔX", String.format("%.1f\"", deltaX));
            telemetry.addData("ΔY", String.format("%.1f\"", deltaY));
            telemetry.addData("ΔHeading", String.format("%.1f°", deltaHeading));

            // === DIAGNOSTICS ===
            telemetry.addData("", "");
            telemetry.addData("═══ DIAGNOSTICS ═══", "");

            if (Math.abs(ftcX_m) < 0.01 && Math.abs(ftcY_m) < 0.01) {
                telemetry.addData("⚠️ ERROR", "Botpose is ZERO!");
                telemetry.addData("  Fix", "Configure AprilTag layout in pipeline");
            }

            if (numTagsDetected == 0) {
                telemetry.addData("⚠️ ERROR", "No tags detected!");
            }

            if (ta < 0.5) {
                telemetry.addData("⚠️ WARNING", "Tag too small - move closer");
            }

            if (Math.abs(tx) > 25.0 || Math.abs(ty) > 25.0) {
                telemetry.addData("⚠️ WARNING", "Tag off-center");
            }

            if (deltaX > 12.0 || deltaY > 12.0) {
                telemetry.addData("💡 TIP", "Large difference - consider relocalization");
            }
        }
        else
        {
            telemetry.addData("Limelight", "❌ NO VALID RESULT");
            telemetry.addData("", "");

            // Still show robot position from odometry
            telemetry.addData("═══ ROBOT POSITION (Odometry) ═══", "");
            telemetry.addData("Pedro X", String.format("%.1f\"", robotX));
            telemetry.addData("Pedro Y", String.format("%.1f\"", robotY));
            telemetry.addData("Pedro Heading", String.format("%.1f°", robotHeading));

            telemetry.addData("", "");
            telemetry.addData("Check:", "");
            telemetry.addData("1", "Is Limelight powered?");
            telemetry.addData("2", "Is pipeline configured?");
            telemetry.addData("3", "Can you see AprilTag?");
        }

        telemetry.update();
    }

    @Override
    public void start()
    {
        limelight.start();
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
