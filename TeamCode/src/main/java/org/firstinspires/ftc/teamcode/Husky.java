package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

// ID 1 - Green, Purple, Purple
// ID 2 - Purple, Green, Purple
// ID 3 - Purple, Purple, Green
// ID 4 - Blue
// ID 5 - Red
public class Husky implements Subsystem
{
    public HuskyLens huskyLens;
    private boolean huskyRead = false;
    private int IdArranged = 0;
    private boolean completeArtifact = false;
    private int IdTeam = 4; // Blue = 4, Red = 5
    public Color[] artifactOrder = {Color.None, Color.None, Color.None};
    String[] arrCol = {"GPP", "PGP", "PPG"};

    private final TelemetryCustom telemetry;
    ButtonReader AlignButton;
    GamepadEx ct1;

    // AprilTag positions on field (inches)
    private static final double BLUE_TAG_X = 17.0;
    private static final double BLUE_TAG_Y = 133.0;
    private static final double RED_TAG_X = 127.0;
    private static final double RED_TAG_Y = 133.0;

    // HuskyLens camera specs
    private static final double HUSKYLENS_FOV_HORIZONTAL = 60.0; // degrees
    private static final int HUSKYLENS_WIDTH = 320;
    private static final int HUSKYLENS_HEIGHT = 240;
    private static final double HUSKYLENS_CENTER_X = HUSKYLENS_WIDTH / 2.0;

    // Camera offset relative to robot center
    private static final double CAMERA_OFFSET_X = 4.52;  // 11cm to the Right
    private static final double CAMERA_OFFSET_Y = -1.53;
    private static final double CAMERA_HEIGHT = 12.48;     // 30cm above ground

    // Tag specs
    private static final double TAG_HEIGHT = 29.13;  // 74cm above ground to center

    // REMOVED THESE LINES - they reference variables that don't exist at class level
    // double normalizedOffset = Math.abs(pixelOffsetX) / (HUSKYLENS_WIDTH / 2.0);
    // double distortionFactor = 1.0 - (0.15 * normalizedOffset);
    // double correctedTagWidth = tagWidth * distortionFactor;
    // double straightLineDistance = K / correctedTagWidth;

    // Store latest tag data
    private int cachedTagX = 0;
    private int cachedTagY = 0;
    private int cachedTagWidth = 0;

    // Constructor with all parameters
    public Husky(TelemetryCustom tl, GamepadEx ct1)
    {
        this.ct1 = ct1;
        this.telemetry = tl;
    }

    // Constructor with just telemetry (for autonomous)
    public Husky(TelemetryCustom tl)
    {
        this.telemetry = tl;
        this.ct1 = null;
    }

    public void LinkComponents(HardwareMap hardwareMap)
    {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
    }

    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.Log("✅ Husky Algorithm", "TAG_RECOGNITION");

        // Only initialize button if gamepad exists (teleop mode)
        if (ct1 != null)
        {
            AlignButton = new ButtonReader(ct1, GamepadKeys.Button.RIGHT_BUMPER);
        }
    }

    public void Run()
    {
        HuskyLens.Block[] blocks = huskyLens.blocks();

        // DEBUG
        telemetry.Log("🔍 Husky Blocks", blocks.length);
        ReadHusky(blocks);
    }

    private void ReadHusky(HuskyLens.Block[] blocuri)
    {
        // ONLY read artifact arrangement once at the beginning
        if(!huskyRead)
        {
            for (HuskyLens.Block block : blocuri)
            {
                if(block.id != 0 && block.id <= 3)
                {
                    IdArranged = block.id;
                    huskyRead = true;
                    telemetry.Log("Arrangement ID", GetID());
                    break;
                }
            }
        }

        if(huskyRead && !completeArtifact)
            CompleteColor(IdArranged);

        // ONLY update team tag cache (never use IDs 1, 2, 3 for relocalization)
        ReadTeamTag(blocuri);
    }

    private void ReadTeamTag(HuskyLens.Block[] blocks)
    {
        for (HuskyLens.Block block : blocks)
        {
            // CRITICAL: Only accept team tags (4 or 5), NEVER artifact tags (1, 2, 3)
            if (block.id == IdTeam)  // This already filters to only 4 or 5
            {
                cachedTagX = block.x;
                cachedTagY = block.y;
                cachedTagWidth = block.width;

                telemetry.Log("✅ Team Tag Found", "ID " + IdTeam);
                telemetry.Log("  Tag X", cachedTagX);
                telemetry.Log("  Tag Width", cachedTagWidth);
                return;  // Exit as soon as we find OUR team tag
            }
        }

        // No TEAM tag found - clear cache
        cachedTagX = 0;
        cachedTagY = 0;
        cachedTagWidth = 0;
    }


    // Public method for RobotAlignment to get latest pose
    public RobotPoseData CalculateRobotPose(double currentHeadingFromIMU)
    {
        // ADDED: Extra safety check - only calculate if we have valid TEAM tag data
        if (cachedTagX == 0 || cachedTagY == 0 || cachedTagWidth == 0) {
            return null;  // No valid team tag detected
        }

        // ADDED: Sanity check - make sure cached data looks reasonable
        if (cachedTagWidth < 10 || cachedTagWidth > 200) {
            telemetry.Log("⚠️ Invalid Width", cachedTagWidth);
            return null;  // Tag width out of reasonable range
        }

        return CalculateRobotPoseInternal(cachedTagX, cachedTagY, cachedTagWidth, currentHeadingFromIMU);
    }

    private RobotPoseData CalculateRobotPoseInternal(int tagX, int tagY, int tagWidth, double headingHint)
    {
        try
        {
            double tagFieldX = (IdTeam == 4) ? BLUE_TAG_X : RED_TAG_X;
            double tagFieldY = (IdTeam == 4) ? BLUE_TAG_Y : RED_TAG_Y;

            telemetry.Log("════════════════════", "");
            telemetry.Log("🏷️ Tag Field Pos", String.format("(%.1f, %.1f)", tagFieldX, tagFieldY));

            // Calculate angle offset
            double pixelOffsetX = tagX - HUSKYLENS_CENTER_X;
            double degreesPerPixel = HUSKYLENS_FOV_HORIZONTAL / HUSKYLENS_WIDTH;
            double angleOffsetDegrees = pixelOffsetX * degreesPerPixel;

            telemetry.Log("📷 Tag in Frame", String.format("x=%d w=%d", tagX, tagWidth));
            telemetry.Log("📷 Pixel Offset", String.format("%.1f px = %.2f°", pixelOffsetX, angleOffsetDegrees));

            // ===== FOV DISTORTION CORRECTION =====
            // Calculate how far from center the tag is (0 = center, 1 = edge)
            double normalizedOffset = Math.abs(pixelOffsetX) / (HUSKYLENS_WIDTH / 2.0);

            // Apply distortion correction (tags appear larger at edges)
            // Reduce width by up to 15% at the edges
            // In CalculateRobotPoseInternal method, change this line:
            double distortionFactor = 1.0 - (0.25 * normalizedOffset);  // Changed from 0.15 to 0.25

            // Corrected width
            double correctedTagWidth = tagWidth * distortionFactor;

            telemetry.Log("📷 Norm Offset", String.format("%.2f", normalizedOffset));
            telemetry.Log("📷 Distortion", String.format("factor=%.3f", distortionFactor));
            telemetry.Log("📷 Width", String.format("raw=%d → %.1f", tagWidth, correctedTagWidth));
            // ===== END DISTORTION CORRECTION =====

            // Calculate distance using CORRECTED width
            double K = 2041;
            double straightLineDistance = K / correctedTagWidth;
            straightLineDistance = Math.max(12.0, Math.min(120.0, straightLineDistance));

            double heightDifference = TAG_HEIGHT - CAMERA_HEIGHT;
            double horizontalDistance = Math.sqrt(
                    straightLineDistance * straightLineDistance -
                            heightDifference * heightDifference
            );

            telemetry.Log("📏 3D Distance", String.format("%.1f\"", straightLineDistance));
            telemetry.Log("📏 Horiz Distance", String.format("%.1f\"", horizontalDistance));

            // USE THE IMU HEADING
            double robotHeading = headingHint;

            // Calculate bearing from camera to tag
            double bearingToTag = robotHeading - angleOffsetDegrees;
            double bearingRad = Math.toRadians(bearingToTag);

            // Camera position
            double cameraX = tagFieldX - horizontalDistance * Math.cos(bearingRad);
            double cameraY = tagFieldY - horizontalDistance * Math.sin(bearingRad);

            telemetry.Log("📐 Robot Heading", String.format("%.1f°", robotHeading));
            telemetry.Log("📐 Bearing→Tag", String.format("%.1f°", bearingToTag));
            telemetry.Log("📷 Camera Pos", String.format("(%.1f, %.1f)", cameraX, cameraY));

            // Apply camera offset
            double headingRad = Math.toRadians(robotHeading);
            double offsetRotatedX = CAMERA_OFFSET_X * Math.cos(headingRad) - CAMERA_OFFSET_Y * Math.sin(headingRad);
            double offsetRotatedY = CAMERA_OFFSET_X * Math.sin(headingRad) + CAMERA_OFFSET_Y * Math.cos(headingRad);

            double robotCenterX = cameraX - offsetRotatedX;
            double robotCenterY = cameraY - offsetRotatedY;

            telemetry.Log("🤖 Robot Pos", String.format("(%.1f, %.1f)", robotCenterX, robotCenterY));
            telemetry.Log("════════════════════", "");

            return new RobotPoseData(robotCenterX, robotCenterY, robotHeading);
        }
        catch (Exception e)
        {
            telemetry.Log("⚠️ Pose Error", e.getMessage());
            return null;
        }
    }

    public int GetID() {return IdArranged;}

    private void CompleteColor(int id)
    {
        completeArtifact = true;
        for(int i = 0; i < arrCol[id - 1].length(); i++)
        {
            char c = arrCol[id - 1].charAt(i);
            artifactOrder[i] = CharToColor(c);
        }
    }

    private Color CharToColor(char c)
    {
        switch(c)
        {
            case 'G':
                return Color.Green;
            case 'P':
                return Color.Purple;
            default:
                return Color.None;
        }
    }

    public void SetTeamId(int teamId)
    {
        if (teamId == 4 || teamId == 5)
            this.IdTeam = teamId;
    }

    public int GetTeamId() {return IdTeam;}
}

// Simple data class to hold robot pose
class RobotPoseData
{
    public double x;
    public double y;
    public double heading;

    public RobotPoseData(double x, double y, double heading)
    {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}