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

public class LimeLight implements Subsystem {
    private Limelight3A limelight;
    private int IdTag = 0;
    private boolean isBlue = true;
    private boolean IsAuto = true;
    private GoBildaPinpointDriver pinpoint;

    // Constante pentru conversie
    private static final double METERS_TO_INCHES = 39.3701;

    // Target positions (in inches, Pedro coordinates)
    private double targetX;
    private double targetY;

    // Robot position from MT2/MT1 (used for calculations)
    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;

    // Pinpoint position (raw odometry)
    private double pinpointX = 0;
    private double pinpointY = 0;
    private double pinpointHeading = 0;
    private double pinpointHeadingVelocity = 0; // Degrees per second

    // Debug flag to disable MT2 orientation updates
    private boolean useMT2OrientationUpdates = true;

    // Configuration: Use MT1 (vision-only) instead of MT2 (vision+IMU fusion)
    // RECOMMENDED: Set to true if you reset robot position during TeleOp!
    // MT1 is more reliable when position resets occur because it doesn't accumulate
    // tracking state.
    private boolean USE_MT1_ONLY = false; // Default: false to enable MT2 (Vision+IMU)

    // MT1 position (camera only)
    private double mt1_X = 0;
    private double mt1_Y = 0;
    private double mt1_Heading = 0;
    private boolean mt1_Valid = false;

    // MT1 RAW (meters, direct from camera)
    private double mt1_raw_X = 0;
    private double mt1_raw_Y = 0;
    private double mt1_raw_Heading = 0;

    // MT2 position (camera + IMU fusion)
    private double mt2_X = 0;
    private double mt2_Y = 0;
    private double mt2_Heading = 0;
    private boolean mt2_Valid = false;

    // MT2 RAW (meters, direct from camera)
    private double mt2_raw_X = 0;
    private double mt2_raw_Y = 0;
    private double mt2_raw_Heading = 0;

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

    public boolean IsBlue() {
        return isBlue;
    }

    public void LinkComponents(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public void Initialize(HardwareMap hardwareMap) {
        LinkComponents(hardwareMap);
        pinpoint.setOffsets(2.11, -3.31);

        // HIGH SPEED TRACKING SETUP
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100); // Max possible update rate
        limelight.start();

        // NOTE: For fast rotation, go to http://limelight.local:5801
        // Set "Exposure" < 2ms and "Gain" High!
    }

    // Update the Run() method:
    public void Run() {

        // Read current Pinpoint position (continuously updated by RobotPinpoint.Run())
        UpdatePinpointPosition();

        UpdateRobotPositionFromLimelight();
        CalculateDistanceToTarget();
        UpdateDistanceToAprilTag();

        // If we haven't detected artifact order yet, use pipeline 0
        if (!artifactOrderDetected) {
            limelight.pipelineSwitch(0); // Detection pipeline for tags 21-23

            // Check if we detected an artifact tag
            if (limelight.getLatestResult() != null && limelight.getLatestResult().isValid()) {
                LLResult llResult = limelight.getLatestResult();
                if (llResult.getFiducialResults() != null && !llResult.getFiducialResults().isEmpty()) {
                    int detectedId = llResult.getFiducialResults().get(0).getFiducialId();

                    // If we detect a tag in range 21-23, save order and switch mode
                    if (detectedId >= 21 && detectedId <= 23) {
                        IdTag = detectedId;
                        SetOrder(IdTag);
                        artifactOrderDetected = true; // Mark as detected, switch pipelines next frame
                    }
                }
            }
        } else {
            // Artifact order detected, now track alliance basket
            if (isBlue)
                RelocalizationBlue(); // Pipeline 2 for blue basket
            else
                RelocalizationRed(); // Pipeline 1 for red basket
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

    /**
     * Citește poziția curentă de la Pinpoint (care se actualizează continuu în
     * RobotPinpoint.Run())
     */
    private void UpdatePinpointPosition() {
        if (pinpoint != null) {
            // ❌ NU apelăm pinpoint.update() - RobotPinpoint.Run() deja face asta
            // Doar citim poziția curentă actualizată

            org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose = pinpoint.getPosition();
            pinpointX = pose.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
            pinpointY = pose.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
            pinpointHeading = Math
                    .toDegrees(pose.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS));
        }
    }

    // Offset to align Pinpoint heading with Limelight Field heading.
    // Adjusted to 84.0 based on MT2 vs MT1 alignment.
    // MT2 (71.2, 102.8) vs MT1 (73.2, 100) - very close, minor tweak.
    private double HEADING_OFFSET_DEG = 90.0;
    private static final double X_AXIS_CALIBRATION_OFFSET = 1.5;
    private static final double Y_AXIS_CALIBRATION_OFFSET = 5.0;
    private double DISTANCE_SCALE_FACTOR = 1.01;

    /**
     * Actualizează orientarea robotului în Limelight folosind IMU de la Pinpoint
     */
    private void UpdateOrientationFromPinpoint() {
        if (USE_MT1_ONLY)
            return;

        if (pinpoint != null && useMT2OrientationUpdates) {
            double rawRadians = pinpoint.getHeading();
            double rawDegrees = Math.toDegrees(rawRadians);
            double correctedDegrees = rawDegrees + HEADING_OFFSET_DEG;

            while (correctedDegrees <= -180)
                correctedDegrees += 360;
            while (correctedDegrees > 180)
                correctedDegrees -= 360;

            limelight.updateRobotOrientation(correctedDegrees);
        }
    }

    /**
     * Calculates the camera position offset based on turret angle.
     *
     * ⚠️ IMPORTANT NOTES:
     * 1. Limelight3A does NOT support dynamic camera pose updates during runtime
     * 2. This method is for REFERENCE ONLY - it's not actually used by Limelight
     * 3. Uses FTC SDK coordinate convention (different from Limelight web config!)
     *
     * The camera pose is configured ONCE in the Limelight web interface:
     * - Go to Settings → Robot → Camera Orientation
     * - Enter Pitch = +15° (Limelight uses standard convention: 0° = horizontal,
     * positive = up)
     * - Enter Yaw = 0° (camera faces forward when turret encoder = 0°)
     * - Enter Roll = 0°
     *
     * This method calculates the camera offset for reference in FTC SDK format,
     * which uses REP-103 convention where:
     * - Pitch 0° = pointing UP (at sky)
     * - Pitch -90° = HORIZONTAL
     * - Pitch -75° = tilted UP 15° from horizontal (your camera)
     *
     * @param turretAngleDeg Turret angle in degrees (0 = forward, positive = CCW)
     * @return Pose3D representing camera position in robot space (FTC SDK format)
     */
    public Pose3D CalculateCameraPoseFromTurret(double turretAngleDeg) {
        double angleRad = Math.toRadians(turretAngleDeg);

        // Calculate camera position in robot coordinate system
        // Camera rotates around turret pivot as turret rotates
        // CameraX = TurretX + cos(angle)*CamTurretX - sin(angle)*CamTurretY
        // CameraY = TurretY + sin(angle)*CamTurretX + cos(angle)*CamTurretY
        double camX = TURRET_ROBOT_X + Math.cos(angleRad) * CAMERA_TURRET_X_AT_ZERO
                - Math.sin(angleRad) * CAMERA_TURRET_Y_AT_ZERO;
        double camY = TURRET_ROBOT_Y + Math.sin(angleRad) * CAMERA_TURRET_X_AT_ZERO
                + Math.cos(angleRad) * CAMERA_TURRET_Y_AT_ZERO;

        // Convert to meters for standard pose representation
        double x_m = camX * 0.0254;
        double y_m = camY * 0.0254;
        double z_m = CAMERA_HEIGHT * 0.0254;

        // Camera orientation in robot space
        // Yaw = turret rotation (camera rotates with turret)
        // Pitch: In FTC coordinates, 0° = pointing up, -90° = horizontal
        // Camera is tilted UP 15° from horizontal → pitch = -90° + 15° = -75°
        // Roll = 0 (camera mounted upright)
        org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles cameraOrientation = new org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles(
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES,
                (float) turretAngleDeg, // Yaw: rotation with turret (0° = forward)
                (float) (-90.0 + CAMERA_PITCH_DEG), // Pitch: -90° (horizontal) + 15° (tilt up) = -75°
                0.0f, // Roll: no roll
                0);

        // Create and return camera pose
        return new Pose3D(
                new org.firstinspires.ftc.robotcore.external.navigation.Position(
                        org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER,
                        x_m, y_m, z_m, 0),
                cameraOrientation);
    }

    /**
     * /**
     * ✅ UPDATED: Relocalizes Pinpoint using MT2 data with stability checks.
     * accounting for turret offset.
     */
    public void RelocalizePinpointWithMT2(double turretAngleDeg) {
        if (!mt2_Valid) {
            mt2StabilityCounter = 0;
            return;
        }

        // 1. Check for stability (oscillation check)
        double drift = Math.sqrt(Math.pow(mt2_X - lastMT2_X, 2) + Math.pow(mt2_Y - lastMT2_Y, 2));
        if (drift < MT2_DRIFT_THRESHOLD) {
            mt2StabilityCounter++;
        } else {
            mt2StabilityCounter = 0;
        }

        lastMT2_X = mt2_X;
        lastMT2_Y = mt2_Y;

        // 2. Only relocalize if stable over multiple frames
        if (mt2StabilityCounter >= MT2_STABILITY_REQUIRED) {
            CompensatePinpointForTurret(turretAngleDeg);
            // mt2StabilityCounter = 0; // Optional: Reset to wait for next stable window
        }
    }

    public void CompensatePinpointForTurret(double turretAngleDeg) {
        if (pinpoint == null)
            return;

        // 1. Update Limelight with absolute camera orientation
        double robotHeadingDegWithOffset = pinpointHeading + HEADING_OFFSET_DEG;
        double cameraAbsoluteHeadingDeg = robotHeadingDegWithOffset + turretAngleDeg;

        while (cameraAbsoluteHeadingDeg <= -180)
            cameraAbsoluteHeadingDeg += 360;
        while (cameraAbsoluteHeadingDeg > 180)
            cameraAbsoluteHeadingDeg -= 360;

        limelight.updateRobotOrientation(cameraAbsoluteHeadingDeg);

        // 2. Calculate camera offset relative to robot center
        double angleRad = Math.toRadians(turretAngleDeg);
        double camX = TURRET_ROBOT_X + Math.cos(angleRad) * CAMERA_TURRET_X_AT_ZERO
                - Math.sin(angleRad) * CAMERA_TURRET_Y_AT_ZERO;
        double camY = TURRET_ROBOT_Y + Math.sin(angleRad) * CAMERA_TURRET_X_AT_ZERO
                + Math.cos(angleRad) * CAMERA_TURRET_Y_AT_ZERO;

        // 3. Compensate Pinpoint
        if (mt2_Valid) {
            // Field-space offset based on robot heading
            double robotHeadingRad = Math.toRadians(pinpointHeading);
            double cosH = Math.cos(robotHeadingRad);
            double sinH = Math.sin(robotHeadingRad);

            double offsetX = camX * cosH - camY * sinH;
            double offsetY = camX * sinH + camY * cosH;

            // Vision Pose (MT2 gives robot center estimation already, but we refine it)
            double compensatedX = mt2_X - offsetX;
            double compensatedY = mt2_Y - offsetY;

            // Apply to Pinpoint (Keep existing heading)
            pinpoint.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                    org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
                    compensatedX,
                    compensatedY,
                    org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES,
                    pinpointHeading));
        }
    }

    /**
     * Actualizează poziția robotului din MT2 și MT1
     * Folosim getBotpose_wpiBlue pentru coordonatele standard FTC
     */
    private void UpdateRobotPositionFromLimelight() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // 1. Update MT1 (Vision Only)
            Pose3D botpose_mt1 = result.getBotpose();
            if (botpose_mt1 != null) {
                mt1_raw_X = botpose_mt1.getPosition().x;
                mt1_raw_Y = botpose_mt1.getPosition().y;
                mt1_raw_Heading = botpose_mt1.getOrientation()
                        .getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);

                Pose mt1_pdf = ConvertToPedroCoordinates(mt1_raw_X, mt1_raw_Y, mt1_raw_Heading);
                mt1_X = mt1_pdf.getX();
                mt1_Y = mt1_pdf.getY();
                mt1_Heading = Math.toDegrees(mt1_pdf.getHeading());
                mt1_Valid = true;
            } else {
                mt1_Valid = false;
            }

            // 2. Update MT2 (Vision + IMU Fusion)
            if (!USE_MT1_ONLY) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    mt2_raw_X = botpose_mt2.getPosition().x;
                    mt2_raw_Y = botpose_mt2.getPosition().y;
                    mt2_raw_Heading = botpose_mt2.getOrientation()
                            .getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);

                    Pose mt2_pdf = ConvertToPedroCoordinates(mt2_raw_X, mt2_raw_Y, mt2_raw_Heading);
                    mt2_X = mt2_pdf.getX();
                    mt2_Y = mt2_pdf.getY();
                    mt2_Heading = Math.toDegrees(mt2_pdf.getHeading());
                    mt2_Valid = true;
                } else {
                    mt2_Valid = false;
                }
            } else {
                mt2_Valid = false;
            }
        } else {
            mt1_Valid = false;
            mt2_Valid = false;
        }

        // 3. Select best available position for robotX, robotY, robotHeading
        if (mt2_Valid && !USE_MT1_ONLY) {
            robotX = mt2_X;
            robotY = mt2_Y;
            robotHeading = Math.toRadians(mt2_Heading);
        } else if (mt1_Valid) {
            robotX = mt1_X;
            robotY = mt1_Y;
            robotHeading = Math.toRadians(mt1_Heading);
        } else {
            // HYBRID FALLBACK: Use Pinpoint (Raw Odometry) if vision is lost
            robotX = pinpointX;
            robotY = pinpointY;
            robotHeading = Math.toRadians(pinpointHeading);
        }
    }

    /**
     * Converts FTC Field Coordinates (Meters, Center Origin) to Pedro Pathing
     * Coordinates (Inches, Corner Origin)
     */
    /**
     * Converts FTC Field Coordinates (Meters, Center Origin) to Pedro Pathing
     * Coordinates (Inches, Corner Origin)
     * Using Community Standard Calculation:
     * 1. Convert to Inches
     * 2. Shift Origin (Center -> Corner)
     * 3. Swap Axes (Field -> Robot/Pedro)
     * 4. Invert Heading
     */
    private Pose ConvertToPedroCoordinates(double x_meters, double y_meters, double heading_degrees) {
        // Convert input meters to inches
        double xFtc = x_meters * METERS_TO_INCHES * DISTANCE_SCALE_FACTOR;
        double yFtc = y_meters * METERS_TO_INCHES * DISTANCE_SCALE_FACTOR;

        // 1) Shift Origin: Center (0,0) -> Corner (72, 72)
        double xShifted = 72.0 - xFtc;
        double yShifted = yFtc + 72.0;

        // 2) Axis Swap (Field X/Y -> Pedro X/Y?)
        double pedroX = yShifted + X_AXIS_CALIBRATION_OFFSET;
        double pedroY = xShifted + Y_AXIS_CALIBRATION_OFFSET;

        // 3) Heading Conversion
        // We subtract the HEADING_OFFSET_DEG to return to the original Pedro/Pinpoint
        // heading space.
        double headingPedroDegrees = heading_degrees - HEADING_OFFSET_DEG;

        // Normalize heading to (-180, 180] to match Pinpoint/Standard FTC style
        while (headingPedroDegrees <= -180)
            headingPedroDegrees += 360;
        while (headingPedroDegrees > 180)
            headingPedroDegrees -= 360;

        return new Pose(pedroX, pedroY, Math.toRadians(headingPedroDegrees));
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

        if (result == null || !result.isValid()) {
            distanceToAprilTagInches = -1;
            return;
        }

        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            distanceToAprilTagInches = -1;
            return;
        }

        // Ia primul tag detectat
        LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);

        // Obține poziția robotului relativ la tag
        Pose3D robotPoseTargetSpace = fiducial.getRobotPoseTargetSpace();

        if (robotPoseTargetSpace == null) {
            distanceToAprilTagInches = -1;
            return;
        }

        // Calculează distanța euclidiană 3D
        double x = robotPoseTargetSpace.getPosition().x;
        double y = robotPoseTargetSpace.getPosition().y;
        double z = robotPoseTargetSpace.getPosition().z;

        double distance_meters = Math.sqrt(x * x + y * y + z * z);

        // Convertește în inches
        distanceToAprilTagInches = distance_meters * METERS_TO_INCHES;
    }

    public int GetID() {
        return IdTag;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public void RelocalizationRed() {
        // Switch to pipeline for red tag
        limelight.pipelineSwitch(1); // Set to the number you assign Red Tag in the Limelight UI
    }

    public void RelocalizationBlue() {
        // Set to the number you assign Blue Tag in the Limelight UI
        limelight.pipelineSwitch(2);
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

    /*
    public double GetDistanceToTrgetPython()
    {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double[] pythonOutputs = result.getPythonOutput();

            if (pythonOutputs != null && pythonOutputs.length > 1)
            {
                double hasTarget = pythonOutputs[0];      // 1 if target found, 0 otherwise
                double tagDistance = pythonOutputs[2];    // Distance to tag in meters
                if (hasTarget == 1) return tagDistance;
            }
        }
        return 3; // No target found
    }
*/
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

        if (result == null || !result.isValid()) {
            return -1;
        }

        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            return -1;
        }

        // Ia primul tag detectat
        LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);

        // Obține poziția robotului relativ la tag (în target space)
        Pose3D robotPoseTargetSpace = fiducial.getRobotPoseTargetSpace();

        if (robotPoseTargetSpace == null) {
            return -1;
        }

        // robotPoseTargetSpace = unde e CAMERA față de TAG
        // În sistemul camerei: X = left/right, Y = up/down, Z = forward/back
        // Pentru distanță 2D ignorăm Y (înălțimea)
        double x = robotPoseTargetSpace.getPosition().x; // Left/Right
        double z = robotPoseTargetSpace.getPosition().z; // Forward/Back

        // Calculează distanța 2D (doar pe planul orizontal)
        double distance_meters_2D = Math.sqrt(x * x + z * z);

        // Convertește în inches
        return distance_meters_2D * METERS_TO_INCHES;
    }

    /**
     * Calculează distanța de la CENTRUL ROBOTULUI la AprilTag, compensând pentru
     * offset-ul turelei.
     *
     * NU depinde de MT2! Folosește doar vision data de la Limelight.
     * Aceasta e distanța pe care ar trebui să o folosești pentru shooter!
     *
     * @param turretAngleDeg Unghiul curent al turelei în grade
     * @return Distanța în inches de la centrul robotului la tag, sau -1 dacă nu
     *         vede tag-ul
     */
    public double GetDistanceToAprilTagFromRobotCenter(double turretAngleDeg) {
        if (distanceToAprilTagInches <= 0) {
            return -1; // Nu vedem tag-ul
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return -1;
        }

        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            return -1;
        }

        LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
        Pose3D robotPoseTargetSpace = fiducial.getRobotPoseTargetSpace();

        if (robotPoseTargetSpace == null) {
            return -1;
        }

        // robotPoseTargetSpace = poziția ROBOTULUI relativ la TAG
        // Trebuie inversată pentru a obține poziția TAG-ului relativ la CAMERĂ
        // În sistemul camerei: forward=Z, left=X, up=Y (standard camera frame)
        // robotPoseTargetSpace dă unde e robotul față de tag, deci inversăm
        double robotX_inTagFrame = robotPoseTargetSpace.getPosition().x;
        double robotY_inTagFrame = robotPoseTargetSpace.getPosition().y;
        double robotZ_inTagFrame = robotPoseTargetSpace.getPosition().z;

        // Tag relativ la cameră = negarea poziției robotului
        double tagX_camera = -robotX_inTagFrame;
        double tagY_camera = -robotY_inTagFrame;
        double tagZ_camera = -robotZ_inTagFrame;

        // Calculăm offset-ul camerei față de centrul robotului (în inches)
        // Camera se rotește în jurul centrului turelei pe măsură ce turreta se rotește
        double angleRad = Math.toRadians(turretAngleDeg);
        double camOffsetX = TURRET_ROBOT_X + Math.cos(angleRad) * CAMERA_TURRET_X_AT_ZERO
                - Math.sin(angleRad) * CAMERA_TURRET_Y_AT_ZERO;
        double camOffsetY = TURRET_ROBOT_Y + Math.sin(angleRad) * CAMERA_TURRET_X_AT_ZERO
                + Math.cos(angleRad) * CAMERA_TURRET_Y_AT_ZERO;

        // Convertim offset-ul în metri
        double camOffsetX_m = camOffsetX * 0.0254;
        double camOffsetY_m = camOffsetY * 0.0254;

        // Poziția tag-ului în sistemul robotului
        // Camera e rotită cu turretAngleDeg față de robot
        // Când turreta rotește CCW (pozitiv), camera rotește CCW cu ea
        // Trebuie să rotim poziția tag-ului cu +unghiul pentru a-l aduce în frame-ul
        // robotului
        double cosAngle = Math.cos(angleRad);
        double sinAngle = Math.sin(angleRad);
        double tagX_robot = tagX_camera * cosAngle - tagY_camera * sinAngle;
        double tagY_robot = tagX_camera * sinAngle + tagY_camera * cosAngle;

        // Adăugăm offset-ul camerei pentru a obține poziția tag-ului relativ la centrul
        // robotului
        double tagX_fromRobotCenter = tagX_robot + camOffsetX_m;
        double tagY_fromRobotCenter = tagY_robot + camOffsetY_m;

        // Calculăm distanța 3D de la centrul robotului la tag
        // Folosim Z-ul direct (înălțimea e constantă indiferent de turret)
        double distance_m = Math.sqrt(
                tagX_fromRobotCenter * tagX_fromRobotCenter +
                        tagY_fromRobotCenter * tagY_fromRobotCenter +
                        tagZ_camera * tagZ_camera);

        // Convertim în inches
        return distance_m * METERS_TO_INCHES;
    }

    /**
     * Calculează distanța 2D (doar X și Y) de la centrul robotului la AprilTag.
     * Useful pentru shooter deoarece ignora diferența de înălțime.
     *
     * NU depinde de MT2!
     *
     * @param turretAngleDeg Unghiul curent al turelei în grade
     * @return Distanța 2D în inches, sau -1 dacă nu vede tag-ul
     */
    public double GetDistance2DToAprilTagFromRobotCenter(double turretAngleDeg) {
        if (distanceToAprilTagInches <= 0) {
            return -1;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return -1;
        }

        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            return -1;
        }

        LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
        Pose3D robotPoseTargetSpace = fiducial.getRobotPoseTargetSpace();

        if (robotPoseTargetSpace == null) {
            return -1;
        }

        // robotPoseTargetSpace = poziția ROBOTULUI relativ la TAG
        // Inversăm pentru a obține poziția TAG-ului relativ la CAMERĂ
        double robotX_inTagFrame = robotPoseTargetSpace.getPosition().x;
        double robotY_inTagFrame = robotPoseTargetSpace.getPosition().y;

        // Tag relativ la cameră = negarea poziției robotului
        double tagX_camera = -robotX_inTagFrame;
        double tagY_camera = -robotY_inTagFrame;

        // Calculăm offset-ul camerei față de centrul robotului (în inches)
        // Camera se rotește în jurul centrului turelei
        double angleRad = Math.toRadians(turretAngleDeg);
        double camOffsetX = TURRET_ROBOT_X + Math.cos(angleRad) * CAMERA_TURRET_X_AT_ZERO
                - Math.sin(angleRad) * CAMERA_TURRET_Y_AT_ZERO;
        double camOffsetY = TURRET_ROBOT_Y + Math.sin(angleRad) * CAMERA_TURRET_X_AT_ZERO
                + Math.cos(angleRad) * CAMERA_TURRET_Y_AT_ZERO;

        // Convertim offset-ul în metri
        double camOffsetX_m = camOffsetX * 0.0254;
        double camOffsetY_m = camOffsetY * 0.0254;

        // Poziția tag-ului în sistemul robotului
        // Când turreta rotește CCW (pozitiv), camera rotește CCW cu ea
        // Rotim cu +unghiul pentru a-l aduce în frame-ul robotului
        double cosAngle = Math.cos(angleRad);
        double sinAngle = Math.sin(angleRad);
        double tagX_robot = tagX_camera * cosAngle - tagY_camera * sinAngle;
        double tagY_robot = tagX_camera * sinAngle + tagY_camera * cosAngle;

        // Adăugăm offset-ul camerei pentru a obține poziția tag-ului relativ la centrul
        // robotului
        double tagX_fromRobotCenter = tagX_robot + camOffsetX_m;
        double tagY_fromRobotCenter = tagY_robot + camOffsetY_m;

        // Calculăm distanța 2D (doar pe planul orizontal)
        double distance_m = Math.sqrt(
                tagX_fromRobotCenter * tagX_fromRobotCenter +
                        tagY_fromRobotCenter * tagY_fromRobotCenter);

        // Convertim în inches
        return distance_m * METERS_TO_INCHES;
    }

    /**
     * Calculează distanța de la poziția MT2 (Limelight) la poziția target (în
     * inches)
     * Folosește poziția din MT2 (vision-corrected), care e mai precisă decât
     * odometry
     */
    public double GetDistanceFromMT2ToTarget() {
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Returnează scaling factor-ul bazat pe raportul dintre distanța Limelight și
     * distanța MT2
     * Folosește formula: constantaShooter * GetDistanceScaleFactor()
     * 
     * Returnează -1 dacă Limelight nu vede tag-ul
     */
    public double GetDistanceScaleFactor() {
        if (distanceToAprilTagInches <= 0) {
            return -1; // Nu avem date de la Limelight
        }

        double mt2Distance = GetDistanceFromMT2ToTarget();

        if (mt2Distance < 0.01) {
            return 1.0; // Evită diviziunea cu zero
        }

        // Raportul dintre distanța Limelight și distanța MT2
        return distanceToAprilTagInches / mt2Distance;
    }

    /**
     * Returnează numărul de tag-uri văzute simultan
     */
    public int GetVisibleTagCount() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return 0;
        }

        if (result.getFiducialResults() == null) {
            return 0;
        }

        return result.getFiducialResults().size();
    }

    /**
     * Verifică dacă MT2 are date valide
     */
    public boolean IsMT2Valid() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        return result.getBotpose_MT2() != null;
    }

    /**
     * Verifică dacă MT1 are date valide
     */
    public boolean IsMT1Valid() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        return result.getBotpose() != null;
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

    public double GetRobotHeadingVelocity() {
        return pinpointHeadingVelocity;
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

    public void UpdateTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("=== DEBUG MT2 ===");
        telemetry.addData("Mode", USE_MT1_ONLY ? "MT1 ONLY (Vision)" : "MT2 (Vision+IMU)");
        telemetry.addData("MT2 Updates", useMT2OrientationUpdates ? "ENABLED" : "DISABLED");
        telemetry.addData("MT1 Status", mt1_Valid ? "VALID" : "INVALID");
        telemetry.addData("MT2 Status", mt2_Valid ? "VALID" : "INVALID");

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            telemetry.addData("LL Latency", "%.0f ms", result.getCaptureLatency() + result.getParseLatency());
        }

        // Critical Debug Value for Tuning
        double rawPinpoint = Math.toDegrees(pinpoint.getHeading());
        double sentToLL = rawPinpoint + HEADING_OFFSET_DEG;
        double diff = sentToLL - mt1_raw_Heading;

        telemetry.addLine("\n=== RAW VALUES (FROM LIMELIGHT) ===");
        telemetry.addData("MT1 Raw X (meters)", "%.3f", mt1_raw_X);
        telemetry.addData("MT1 Raw Y (meters)", "%.3f", mt1_raw_Y);
        telemetry.addData("MT1 Raw H (deg)", "%.1f°", mt1_raw_Heading);

        telemetry.addData("MT2 Raw X (meters)", "%.3f", mt2_raw_X);
        telemetry.addData("MT2 Raw Y (meters)", "%.3f", mt2_raw_Y);
        telemetry.addData("MT2 Raw H (deg)", "%.1f°", mt2_raw_Heading);

        telemetry.addLine("\n=== HEADING DEBUG ===");
        telemetry.addData("Pinpoint Raw H", "%.1f°", rawPinpoint);
        telemetry.addData("Offset Applied", "%.1f°", HEADING_OFFSET_DEG);
        telemetry.addData("Sent to Limelight", "%.1f°", sentToLL);
        telemetry.addData("DIFF (Tune Offset)", "%.1f°", diff);

        telemetry.addLine("\n=== PEDRO COORDINATES ===");
        if (mt1_Valid) {
            telemetry.addData("MT1 X", "%.1f\"", mt1_X);
            telemetry.addData("MT1 Y", "%.1f\"", mt1_Y);
            telemetry.addData("MT1 H", "%.1f°", mt1_Heading);
        }

        if (mt2_Valid) {
            telemetry.addData("MT2 X", "%.1f\"", mt2_X);
            telemetry.addData("MT2 Y", "%.1f\"", mt2_Y);
            telemetry.addData("MT2 H", "%.1f°", mt2_Heading);
        }

        telemetry.addLine("\n=== PINPOINT POSITION ===");
        telemetry.addData("Pinpoint X", "%.1f\"", pinpointX);
        telemetry.addData("Pinpoint Y", "%.1f\"", pinpointY);
        telemetry.addData("Pinpoint H", "%.1f°", pinpointHeading);

        telemetry.addLine("\n=== DISTANCE TEST ===");
        double mt2Dist = GetDistanceFromMT2ToTarget();
        telemetry.addData("MT2→Target", "%.1f\"", mt2Dist);

        if (distanceToAprilTagInches > 0) {
            telemetry.addData("Limelight→Tag", "%.1f\"", distanceToAprilTagInches);
            double scaleFactor = GetDistanceScaleFactor();
            telemetry.addData("Scale Factor", "%.3f", scaleFactor);
            telemetry.addData("Formula", "consDist * %.3f", scaleFactor);
        } else {
            telemetry.addData("Limelight→Tag", "NOT VISIBLE");
        }

        telemetry.addLine("\n=== TARGET INFO ===");
        telemetry.addData("Target X", "%.1f\"", targetX);
        telemetry.addData("Target Y", "%.1f\"", targetY);
    }

    /**
     * Enable or disable MT2 orientation updates for debugging
     * When disabled, MT2 won't receive orientation data and should behave more like
     * MT1
     */
    public void setUseMT2OrientationUpdates(boolean enable) {
        this.useMT2OrientationUpdates = enable;
    }

    public double getHeadingOffset() {
        return HEADING_OFFSET_DEG;
    }

    public void setHeadingOffset(double offset) {
        this.HEADING_OFFSET_DEG = offset;
    }

    /**
     * Get whether we're using MT1 only (vision-only localization)
     */
    public boolean isUseMT1Only() {
        return USE_MT1_ONLY;
    }

    /**
     * Set whether to use MT1 only (vision-only) or MT2 (vision+IMU fusion)
     * Set to true if you reset robot position during TeleOp!
     */
    public void setUseMT1Only(boolean useMT1Only) {
        this.USE_MT1_ONLY = useMT1Only;
    }
}
