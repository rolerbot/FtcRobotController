package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.PinpointBlocksDriver.GoBildaPinpointDriver;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class LimeLight implements Subsystem {
    private Limelight3A limelight;
    private int IdTag = 0;
    private boolean isBlue = true;
    private boolean IsAuto = true;
    private GoBildaPinpointDriver pinpoint;
    private GamepadEx ct2;
    private ButtonReader btn21, btn22, btn23;
    private int currentPipeline = -1; // Track current pipeline to avoid spamming resets

    // Constante pentru conversie
    private static final double METERS_TO_INCHES = 39.3701;

    // Target positions (in inches, Pedro coordinates)
    private double targetX;
    private double targetY;

    // Simplified Robot position (strictly from Pinpoint)
    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;

    // Distance to target
    private double distanceToTargetMeters = 0;

    // Distance to detected AprilTag
    private double distanceToAprilTagInches = 0;

    // ========== TURRET & CAMERA PHYSICAL MEASUREMENTS (INCHES) ==========
    // MEASURED VALUES FROM PHYSICAL ROBOT (converted from millimeters)

    // 1. TURRET CENTER offset from ROBOT CENTER (in robot's coordinate frame)
    // - Positive X = forward (front of robot)
    // - Positive Y = left (left side of robot)
    // - These values are CONSTANT (turret pivot point doesn't move)
    private static final double TURRET_ROBOT_X = 3.9713; // 100.87 mm - Forward from robot center to turret pivot
    private static final double TURRET_ROBOT_Y = 0.0913; // 2.318 mm - Left from robot center to turret pivot

    // 2. CAMERA offset from TURRET CENTER when turret encoder = 0°
    // - These are measured in the TURRET's local frame at 0°
    // - When turret encoder = 0°, camera faces FORWARD (same heading as robot/IMU)
    // - Positive X = forward relative to turret (away from turret pivot)
    // - Positive Y = left relative to turret
    // - The camera ROTATES with the turret (same yaw angle as turret encoder)
    private static final double CAMERA_TURRET_X_AT_ZERO = 6.3295; // 160.77 mm - Forward from turret center
    private static final double CAMERA_TURRET_Y_AT_ZERO = -0.0083; // -0.21 mm - Slightly RIGHT from turret center

    // 3. CAMERA HEIGHT from ground
    private static final double CAMERA_HEIGHT = 12.8032; // 325.203 mm - Camera height from floor

    // 4. CAMERA PITCH (tilt angle)
    // - This is the PHYSICAL tilt angle: camera is tilted UP 15° from horizontal
    //
    // ⚠️ IMPORTANT: Different systems use different conventions!
    //
    // In LIMELIGHT WEB CONFIG (Settings → Robot → Camera Orientation):
    // Enter: Pitch = +15° (Limelight convention: 0° = horizontal, + = up)
    //
    // In FTC SDK CODE (YawPitchRollAngles):
    // Use: Pitch = -75° (FTC convention: 0° = up, -90° = horizontal)
    // Calculation: -90° (horizontal) + 15° (tilt up) = -75°
    private static final double CAMERA_PITCH_DEG = 15.0; // Physical mounting: tilted UP 15° from horizontal

    // NOTE: The turret diameter is NOT needed for calculations - we only care about
    // the camera's position relative to the turret's CENTER (pivot point).

    public Color[] artifactOrder = { Color.None, Color.None, Color.None };
    private boolean artifactOrderDetected = false;

    // Stability tracking for MT2 relocalization
    private double lastMT2_X = 0;
    private double lastMT2_Y = 0;
    private int mt2StabilityCounter = 0;
    private final int MT2_STABILITY_REQUIRED = 5; // Number of stable frames required
    private final double MT2_DRIFT_THRESHOLD = 0.5; // Max inches of drift to consider "stable"

    public LimeLight(boolean IsBlue, boolean isAuto) {
        this.isBlue = IsBlue;
        this.IsAuto = isAuto;

        // Set target position based on alliance
        if (isBlue) {
            targetX = 0;
            targetY = 144;
        } else {
            // Red alliance basket position
            targetX = 144;
            targetY = 144;
        }
    }

    public LimeLight(boolean IsBlue, boolean isAuto, GamepadEx ct2) {
        this.isBlue = IsBlue;
        this.IsAuto = isAuto;
        this.ct2 = ct2;

        // Set target position based on alliance
        if (isBlue) {
            targetX = 0;
            targetY = 144;
        } else {
            // Red alliance basket position
            targetX = 144;
            targetY = 144;
        }
    }

    public boolean IsBlue() {
        return isBlue;
    }

    public void LinkComponents(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public void ForceShootingPipeline() {
        this.artifactOrderDetected = true;
        if (isBlue)
            RelocalizationBlue();
        else
            RelocalizationRed();
    }

    public void Initialize(HardwareMap hardwareMap) {
        LinkComponents(hardwareMap);
        pinpoint.setOffsets(2.11, -3.31);

        // HIGH SPEED TRACKING SETUP
        limelight.setPollRateHz(125); // Max possible update rate
        limelight.start();

        if (ct2 != null) {
            btn21 = new ButtonReader(ct2, GamepadKeys.Button.Y);
            btn22 = new ButtonReader(ct2, GamepadKeys.Button.X);
            btn23 = new ButtonReader(ct2, GamepadKeys.Button.A);
        }

        // Always start on Pipeline 0 to search for artifacts (21, 22, 23)
        switchPipeline(0);
    }

    // Update the Run() method:
    public void Run() {
        UpdateRobotPositionFromPinpoint();

        if (ct2 != null) {
            btn21.readValue();
            btn22.readValue();
            btn23.readValue();

            if (btn21.wasJustPressed()) {
                this.IdTag = 21;
                SetOrder(21);
                this.artifactOrderDetected = true;
            } else if (btn22.wasJustPressed()) {
                this.IdTag = 22;
                SetOrder(22);
                this.artifactOrderDetected = true;
            } else if (btn23.wasJustPressed()) {
                this.IdTag = 23;
                SetOrder(23);
                this.artifactOrderDetected = true;
            }
        }

        if (artifactOrderDetected) {
            CalculateDistanceToTarget();
            UpdateDistanceToAprilTag();

            if (isBlue)
                switchPipeline(2);
            else
                switchPipeline(1);
        } else {
            switchPipeline(0);

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                double[] pythonOut = result.getPythonOutput();
                if (pythonOut != null && pythonOut.length >= 2 && pythonOut[0] > 0.5) {
                    int id = (int) pythonOut[1];
                    if (id == 21 || id == 22 || id == 23) {
                        this.IdTag = id;
                        SetOrder(id);
                        this.artifactOrderDetected = true;
                    }
                }
            }
        }
    }

    private void UpdateRobotPositionFromPinpoint() {
        if (pinpoint != null) {
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose = pinpoint.getPosition();
            robotX = pose.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
            robotY = pose.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
            robotHeading = pose.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);
        }
    }

    private void switchPipeline(int p) {
        if (currentPipeline != p) {
            limelight.pipelineSwitch(p);
            currentPipeline = p;
        }
    }

    // Add a method to reset if you need to detect artifacts again (optional):
    public void ResetArtifactDetection() {
        artifactOrderDetected = false;
        IdTag = 0;
        artifactOrder[0] = Color.None;
        artifactOrder[1] = Color.None;
        artifactOrder[2] = Color.None;
    }

    /**
     * Forces the LimeLight to stop searching for artifact tags (Pipeline 0)
     * and proceed to basket relocalization. Use this for timeouts.
     */
    public void SkipArtifactDetection() {
        artifactOrderDetected = true;
        SetOrder(21);
    }

    private void SetOrder(int id) {
        if (id == 21) {
            artifactOrder[0] = Color.Green;
            artifactOrder[1] = Color.Purple;
            artifactOrder[2] = Color.Purple;
            return;
        }
        if (id == 22) {
            artifactOrder[0] = Color.Purple;
            artifactOrder[1] = Color.Green;
            artifactOrder[2] = Color.Purple;
            return;
        }
        if (id == 23) {
            artifactOrder[0] = Color.Purple;
            artifactOrder[1] = Color.Purple;
            artifactOrder[2] = Color.Green;
        }
    }

    public double GetTargetX() {
        return targetX;
    }

    public double GetTargetY() {
        return targetY;
    }

    /**
     * Calculează distanța până la AprilTag-ul detectat
     */
    private void UpdateDistanceToAprilTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            distanceToAprilTagInches = -1;
            return;
        }
        LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
        Pose3D robotPoseTargetSpace = fiducial.getRobotPoseTargetSpace();
        if (robotPoseTargetSpace == null) {
            distanceToAprilTagInches = -1;
            return;
        }
        double x = robotPoseTargetSpace.getPosition().x;
        double y = robotPoseTargetSpace.getPosition().y;
        double z = robotPoseTargetSpace.getPosition().z;
        distanceToAprilTagInches = Math.sqrt(x * x + y * y + z * z) * METERS_TO_INCHES;
    }

    public int GetID() {
        return IdTag;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public void RelocalizationRed() {
        switchPipeline(1);
    }

    public void RelocalizationBlue() {
        switchPipeline(2);
    }

    private void CalculateDistanceToTarget() {
        // Calculate delta
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;

        // Calculate distance in inches
        double distanceInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Convert to meters
        distanceToTargetMeters = distanceInches * 0.0254;
    }

    public double GetDistanceToTargetPython() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            double[] pythonOutputs = result.getPythonOutput();

            if (pythonOutputs != null && pythonOutputs.length >= 4) {
                if (pythonOutputs[0] > 0.5) {
                    return pythonOutputs[3];
                }
            }
        }
        return -1.0; // Return -1.0 to signal "no target" so Shooter can keep last valid distance
    }

    public double GetDistanceToTarget() {
        return distanceToTargetMeters;
    }

    public double GetDistanceToTargetInches() {
        return distanceToTargetMeters / 0.0254;
    }

    /**
     * Returnează distanța până la AprilTag-ul detectat (în inches)
     * ATENȚIE: Aceasta e distanța de la CAMERĂ la tag, nu de la centrul robotului!
     * Returnează -1 dacă nu e detectat niciun tag
     */
    public double GetDistanceToAprilTag() {
        return distanceToAprilTagInches;
    }

    /**
     * ✅ Calculează distanța 2D (doar X și Y) de la CAMERĂ direct la AprilTag.
     *
     * Ignoră componenta Z (înălțimea) - perfectă pentru calcule de shooter pe plan
     * orizontal.
     * NU compensează pentru poziția robotului sau turetă - distanță PURĂ de la
     * cameră!
     *
     * @return Distanța 2D în inches de la cameră la tag, sau -1 dacă nu vede tag-ul
     */
    public double GetDistance2DToAprilTagFromCamera() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty())
            return -1;
        LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
        Pose3D pose = fiducial.getRobotPoseTargetSpace();
        if (pose == null)
            return -1;
        return Math.sqrt(Math.pow(pose.getPosition().x, 2) + Math.pow(pose.getPosition().z, 2)) * METERS_TO_INCHES;
    }

    public void SetTargetPosition(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public double GetRobotX() {
        return robotX;
    }

    public double GetRobotY() {
        return robotY;
    }

    public double GetRobotHeading() {
        return robotHeading;
    }

    /**
     * Returnează poziția robotului ca Pose pentru Pedro Pathing
     */
    public Pose GetRobotPose() {
        return new Pose(robotX, robotY, robotHeading);
    }

    /**
     * Afișează telemetria pentru Pinpoint, MT1 și MT2
     * Apelează această funcție în TeleOp după limelight.Run()
     */

}
