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

    public Color[] artifactOrder = {Color.None, Color.None, Color.None};

    public LimeLight(boolean IsBlue, boolean isAuto)
    {
        this.isBlue = IsBlue;
        this.IsAuto = isAuto;

        // Set target position based on alliance
        if(isBlue) {
            // Blue alliance basket position
            targetX = 0;
            targetY = 143.6;
        } else {
            // Red alliance basket position
            targetX = 143.6;
            targetY = 143.6;
        }
    }

    public boolean IsBlue() {return isBlue;}

    public void LinkComponents(HardwareMap hardwareMap)
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
        // Update Pinpoint position (raw odometry)
        UpdatePinpointPosition();

        // Update robot orientation from Pinpoint IMU for MT2
        UpdateOrientationFromPinpoint();

        // Update robot position from MT2 and MT1
        UpdateRobotPositionFromLimelight();

        if(isBlue && IdTag == 0)
            RelocalizationBlue();
        else if(IdTag == 0)
            RelocalizationRed();

        CalculateDistanceToTarget();
        UpdateDistanceToAprilTag();

        if(limelight.getLatestResult() != null && limelight.getLatestResult().isValid() && IdTag == 0)
        {
            LLResult llResult = limelight.getLatestResult();
            if (llResult.getFiducialResults() != null && !llResult.getFiducialResults().isEmpty()) {
                IdTag = llResult.getFiducialResults().get(0).getFiducialId();
            }
        }
    }

    /**
     * Actualizează poziția de la Pinpoint (odometry pur)
     */
    private void UpdatePinpointPosition() {
        if (pinpoint != null) {
            pinpoint.update();
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose = pinpoint.getPosition();
            pinpointX = pose.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
            pinpointY = pose.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
            pinpointHeading = Math.toDegrees(pose.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS));
        }
    }

    // Offset to align Pinpoint heading with Limelight Field heading.
    // New Calculation swaps X/Y, so we might need a 90 degree offset for that too.
    // Try +90.0 first with this new rotation logic.
    private double HEADING_OFFSET_DEG = 90.0;

    /**
     * Actualizează orientarea robotului în Limelight folosind IMU de la Pinpoint
     * Necesar pentru MT2!
     */
    private void UpdateOrientationFromPinpoint() {
        if (pinpoint != null) {
            // Get raw heading in Radians
            double rawRadians = pinpoint.getHeading();
            
            // Apply offset (converted to radians)
            double offsetRadians = Math.toRadians(HEADING_OFFSET_DEG);
            double correctedRadians = rawRadians + offsetRadians;
            
            limelight.updateRobotOrientation(correctedRadians);
        }
    }

    /**
     * Actualizează poziția robotului din MT2 și MT1
     * Folosim getBotpose_wpiBlue pentru coordonatele standard FTC
     */
    private void UpdateRobotPositionFromLimelight() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            mt1_Valid = false;
            mt2_Valid = false;
            return;
        }

        // Actualizează MT2 (camera + IMU fusion)
        Pose3D botpose_mt2 = result.getBotpose_MT2();
        if (botpose_mt2 != null) {
            // Store RAW values (meters, direct from camera - FTC Field Coordinates)
            mt2_raw_X = botpose_mt2.getPosition().x;
            mt2_raw_Y = botpose_mt2.getPosition().y;
            mt2_raw_Heading = botpose_mt2.getOrientation().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);

            // Convert to Pedro Coordinates (Corner Origin, Inches) using helper
            Pose mt2_pdf = ConvertToPedroCoordinates(mt2_raw_X, mt2_raw_Y, mt2_raw_Heading);

            mt2_X = mt2_pdf.getX();
            mt2_Y = mt2_pdf.getY();
            mt2_Heading = Math.toDegrees(mt2_pdf.getHeading());
            mt2_Valid = true;

            // Folosește MT2 pentru calcule (mai precis)
            robotX = mt2_X;
            robotY = mt2_Y;
            robotHeading = mt2_pdf.getHeading(); // Radians for internal use if compatible with other systems
        } else {
            mt2_Valid = false;
        }

        // Actualizează MT1 (camera only)
        Pose3D botpose_mt1 = result.getBotpose();
        if (botpose_mt1 != null) {
            // Store RAW values (meters, direct from camera - FTC Field Coordinates)
            mt1_raw_X = botpose_mt1.getPosition().x;
            mt1_raw_Y = botpose_mt1.getPosition().y;
            mt1_raw_Heading = botpose_mt1.getOrientation().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);

            // Convert to Pedro Coordinates
            Pose mt1_pdf = ConvertToPedroCoordinates(mt1_raw_X, mt1_raw_Y, mt1_raw_Heading);

            mt1_X = mt1_pdf.getX();
            mt1_Y = mt1_pdf.getY();
            mt1_Heading = Math.toDegrees(mt1_pdf.getHeading());
            mt1_Valid = true;

            // Dacă MT2 nu e disponibil, folosește MT1
            if (!mt2_Valid) {
                robotX = mt1_X;
                robotY = mt1_Y;
                robotHeading = mt1_pdf.getHeading();
            }
        } else {
            mt1_Valid = false;
        }
    }

    /**
     * Converts FTC Field Coordinates (Meters, Center Origin) to Pedro Pathing Coordinates (Inches, Corner Origin)
     */
    /**
     * Converts FTC Field Coordinates (Meters, Center Origin) to Pedro Pathing Coordinates (Inches, Corner Origin)
     * Using Community Standard Calculation:
     * 1. Convert to Inches
     * 2. Shift Origin (Center -> Corner)
     * 3. Swap Axes (Field -> Robot/Pedro)
     * 4. Invert Heading
     */
    private Pose ConvertToPedroCoordinates(double x_meters, double y_meters, double heading_degrees) {
        // Convert input meters to inches
        double xFtc = x_meters * METERS_TO_INCHES;
        double yFtc = y_meters * METERS_TO_INCHES;

        // 1) Shift Origin: Center (0,0) -> Corner (72, 72)
        //    X is inverted relative to corner in this formula? 
        //    Wait, the snippet says: xShifted = 72 - xFtc;
        //    This implies xFtc increases OPPOSITE to Pedro X direction.
        double xShifted = 72.0 - xFtc;  // Invert X and shift
        double yShifted = yFtc + 72.0; // Shift Y

        // 2) Axis Swap (Field X/Y -> Pedro X/Y?)
        //    Snippet: xPedro = yShifted, yPedro = xShifted
        //    This rotates the system 90 degrees.
        double pedroX = yShifted;
        double pedroY = xShifted;

        // 3) Heading Conversion
        //    Snippet: headingPedro = -headingRad
        //    We have degrees here, so:
        double headingPedroDegrees = -heading_degrees;

        // Note: The community snippet likely assumes a specific field orientation.
        // If your X/Y are swapped, this Axis Swap (Step 2) fixes it.

        return new Pose(pedroX, pedroY, Math.toRadians(headingPedroDegrees));
    }

    public double GetTargetX() {return targetX;}
    public double GetTargetY() {return targetY;}

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

        double distance_meters = Math.sqrt(x*x + y*y + z*z);

        // Convertește în inches
        distanceToAprilTagInches = distance_meters * METERS_TO_INCHES;
    }

    public int GetID() {return IdTag;}

    public Limelight3A getLimelight() {return limelight;}

    public void RelocalizationRed()
    {
        // Switch to pipeline for red tag
        limelight.pipelineSwitch(1); // Set to the number you assign Red Tag in the Limelight UI
    }

    public void RelocalizationBlue()
    {
        // Set to the number you assign Blue Tag in the Limelight UI
        limelight.pipelineSwitch(2);
    }

    private void CalculateDistanceToTarget()
    {
        // Calculate delta
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;

        // Calculate distance in inches
        double distanceInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Convert to meters
        distanceToTargetMeters = distanceInches * 0.0254;
    }

    public double GetDistanceToTarget() {return distanceToTargetMeters;}
    public double GetDistanceToTargetInches() {return distanceToTargetMeters / 0.0254;}

    /**
     * Returnează distanța până la AprilTag-ul detectat (în inches)
     * Returnează -1 dacă nu e detectat niciun tag
     */
    public double GetDistanceToAprilTag() {return distanceToAprilTagInches;}

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

    public void SetTargetPosition(double x, double y) {this.targetX = x;this.targetY = y;}

    public double GetRobotX() {return robotX;}

    public double GetRobotY() {return robotY;}

    public double GetRobotHeading() {return robotHeading;}

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
        
        // Critical Debug Value for Tuning
        double rawPinpoint = Math.toDegrees(pinpoint.getHeading());
        double sentToLL = rawPinpoint + HEADING_OFFSET_DEG;
        double diff = sentToLL - mt1_raw_Heading;
        
        telemetry.addData("DIFF (Adjust Offset by this)", "%.1f°", diff);
        telemetry.addData("MT2 Status", mt2_Valid ? "VALID" : "INVALID");

        telemetry.addLine("\n=== COORDINATES ===");
        if (mt2_Valid) {
            telemetry.addData("MT2 X", "%.1f\"", mt2_X);
            telemetry.addData("MT2 Y", "%.1f\"", mt2_Y);
            telemetry.addData("MT2 H", "%.1f°", mt2_Heading);
        } else if (mt1_Valid) {
            telemetry.addData("MT1 X", "%.1f\" (Backup)", mt1_X);
            telemetry.addData("MT1 Y", "%.1f\" (Backup)", mt1_Y);
        } else {
            telemetry.addData("Pos", "NO DATA");
        }
        
        telemetry.addLine("\n=== TARGET INFO ===");
        if (distanceToAprilTagInches > 0) {
            telemetry.addData("Dist to Tag", "%.1f\"", distanceToAprilTagInches);
        }
    }
}

