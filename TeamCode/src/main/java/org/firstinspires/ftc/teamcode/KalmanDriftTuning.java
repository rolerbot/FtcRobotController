package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

/**
 * Kalman Filter Drift Tuning TeleOp
 *
 * This OpMode helps you:
 * 1. Measure pinpoint drift while stationary with flywheel running
 * 2. Calculate process noise (Q matrix) for Kalman filter
 * 3. Calculate sensor noise (R matrix)
 * 4. Test the Kalman filter with live visualization
 *
 * Instructions:
 * 1. Start with robot stationary, flywheel OFF
 * 2. Press DPAD_UP to start collecting drift samples
 * 3. Turn on flywheel
 * 4. Wait 60+ seconds to collect data
 * 5. Press DPAD_DOWN to calculate drift parameters
 * 6. Press X to enable Kalman filter
 * 7. Observe corrected vs raw position
 */
@TeleOp(name = "Kalman Drift Tuning", group = "Tuning")
public class KalmanDriftTuning extends OpMode
{
    private Limelight3A limelight;
    private Follower follower;

    // Data collection
    private ArrayList<DriftSample> driftSamples;
    private ElapsedTime sampleTimer;
    private boolean collectingData = false;
    private double lastSampleX = 0, lastSampleY = 0, lastSampleH = 0;

    // Calculated noise parameters
    private double sigmaX_drift = 0;  // Process noise for X drift
    private double sigmaY_drift = 0;  // Process noise for Y drift
    private double sigmaH_drift = 0;  // Process noise for heading drift
    private double sigmaX_sensor = 0; // Sensor noise for X (pinpoint)
    private double sigmaY_sensor = 0; // Sensor noise for Y
    private double sigmaH_sensor = 0; // Sensor noise for heading
    private double sigmaX_limelight = 0; // Limelight noise X
    private double sigmaY_limelight = 0; // Limelight noise Y
    private double sigmaH_limelight = 0; // Limelight noise heading

    // Kalman filter state (1D simplified version per axis)
    private double driftX = 0;  // Estimated drift in X
    private double driftY = 0;  // Estimated drift in Y
    private double driftH = 0;  // Estimated drift in heading
    private double P_x = 0.5;   // Uncertainty in X drift
    private double P_y = 0.5;   // Uncertainty in Y drift
    private double P_h = 0.1;   // Uncertainty in heading drift

    private boolean filterEnabled = false;
    private double loopTime = 0.02; // 20ms default, will be measured
    private ElapsedTime loopTimer;

    // Initial pose when starting data collection
    private double initialX = 0, initialY = 0, initialH = 0;

    @Override
    public void init()
    {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        // Initialize Pedro Pathing follower (includes Pinpoint)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0)); // Center of field

        // Initialize data collection
        driftSamples = new ArrayList<>();
        sampleTimer = new ElapsedTime();
        loopTimer = new ElapsedTime();

        telemetry.addLine("═══ KALMAN FILTER TUNING ═══");
        telemetry.addLine();
        telemetry.addLine("DPAD_UP: Start drift collection");
        telemetry.addLine("DPAD_DOWN: Calculate parameters");
        telemetry.addLine("X: Enable/disable filter");
        telemetry.addLine("A: Reset drift estimate");
        telemetry.addLine();
        telemetry.addLine("Keep robot STATIONARY");
        telemetry.addLine("Turn on FLYWHEEL for realistic drift");
        telemetry.update();
    }

    @Override
    public void start()
    {
        limelight.start();
        loopTimer.reset();
    }

    @Override
    public void loop()
    {
        // Measure actual loop time
        loopTime = loopTimer.seconds();
        loopTimer.reset();

        follower.update();

        // Get current pose from Pinpoint
        Pose robotPose = follower.getPose();
        double rawX = robotPose.getX();
        double rawY = robotPose.getY();
        double rawH = Math.toDegrees(robotPose.getHeading());

        // Handle button inputs
        handleButtons(rawX, rawY, rawH);

        // Collect drift samples if active
        if (collectingData) {
            collectDriftSamples(rawX, rawY, rawH);
        }

        // Run Kalman filter if enabled
        double correctedX = rawX;
        double correctedY = rawY;
        double correctedH = rawH;

        if (filterEnabled && sigmaX_drift > 0) {
            // Predict step (increase uncertainty)
            double Q_x = sigmaX_drift * sigmaX_drift * loopTime;
            double Q_y = sigmaY_drift * sigmaY_drift * loopTime;
            double Q_h = sigmaH_drift * sigmaH_drift * loopTime;

            P_x += Q_x;
            P_y += Q_y;
            P_h += Q_h;

            // Update step (if Limelight data available)
            updateKalmanFilter(rawX, rawY, rawH);

            // Get corrected position
            correctedX = rawX - driftX;
            correctedY = rawY - driftY;
            correctedH = rawH - driftH;
        }

        // Display telemetry
        displayTelemetry(rawX, rawY, rawH, correctedX, correctedY, correctedH);
    }

    private void handleButtons(double rawX, double rawY, double rawH)
    {
        // Start data collection
        if (gamepad1.dpad_up && !collectingData) {
            startDataCollection(rawX, rawY, rawH);
        }

        // Stop and calculate parameters
        if (gamepad1.dpad_down && collectingData) {
            stopAndCalculate();
        }

        // Toggle filter
        if (gamepad1.x) {
            filterEnabled = !filterEnabled;
            while (gamepad1.x) { /* Wait for release */ }
        }

        // Reset drift estimate
        if (gamepad1.a) {
            driftX = 0;
            driftY = 0;
            driftH = 0;
            P_x = 0.5;
            P_y = 0.5;
            P_h = 0.1;
        }
    }

    private void startDataCollection(double x, double y, double h)
    {
        collectingData = true;
        driftSamples.clear();
        sampleTimer.reset();
        initialX = x;
        initialY = y;
        initialH = h;
        lastSampleX = x;
        lastSampleY = y;
        lastSampleH = h;

        // Add first sample
        driftSamples.add(new DriftSample(0, x - initialX, y - initialY, h - initialH));
    }

    private void collectDriftSamples(double x, double y, double h)
    {
        // Collect one sample per second
        if (sampleTimer.seconds() >= 1.0) {
            double time = driftSamples.size();
            double driftX = x - initialX;
            double driftY = y - initialY;
            double driftH = h - initialH;

            driftSamples.add(new DriftSample(time, driftX, driftY, driftH));

            // Calculate instantaneous sensor noise (variance while "stationary")
            if (driftSamples.size() > 1) {
                double deltaX = x - lastSampleX;
                double deltaY = y - lastSampleY;
                double deltaH = h - lastSampleH;

                // Running variance calculation
                sigmaX_sensor = Math.sqrt((sigmaX_sensor * sigmaX_sensor * (driftSamples.size() - 2) + deltaX * deltaX) / (driftSamples.size() - 1));
                sigmaY_sensor = Math.sqrt((sigmaY_sensor * sigmaY_sensor * (driftSamples.size() - 2) + deltaY * deltaY) / (driftSamples.size() - 1));
                sigmaH_sensor = Math.sqrt((sigmaH_sensor * sigmaH_sensor * (driftSamples.size() - 2) + deltaH * deltaH) / (driftSamples.size() - 1));
            }

            lastSampleX = x;
            lastSampleY = y;
            lastSampleH = h;

            sampleTimer.reset();
        }
    }

    private void stopAndCalculate()
    {
        collectingData = false;

        if (driftSamples.size() < 10) {
            telemetry.addLine("ERROR: Need at least 10 samples!");
            return;
        }

        // Calculate drift random walk standard deviation
        // Method: Compare samples at different time intervals
        ArrayList<Double> sigmasX = new ArrayList<>();
        ArrayList<Double> sigmasY = new ArrayList<>();
        ArrayList<Double> sigmasH = new ArrayList<>();

        // Try different time intervals (1s, 5s, 10s, etc.)
        int[] intervals = {1, 5, 10, 20};

        for (int interval : intervals) {
            if (driftSamples.size() <= interval) continue;

            double sumX = 0, sumY = 0, sumH = 0;
            int count = 0;

            for (int i = interval; i < driftSamples.size(); i++) {
                DriftSample s1 = driftSamples.get(i - interval);
                DriftSample s2 = driftSamples.get(i);

                double deltaX = Math.abs(s2.driftX - s1.driftX);
                double deltaY = Math.abs(s2.driftY - s1.driftY);
                double deltaH = Math.abs(s2.driftH - s1.driftH);

                sumX += deltaX;
                sumY += deltaY;
                sumH += deltaH;
                count++;
            }

            // Average difference / sqrt(time)
            double avgX = (sumX / count) / Math.sqrt(interval);
            double avgY = (sumY / count) / Math.sqrt(interval);
            double avgH = (sumH / count) / Math.sqrt(interval);

            sigmasX.add(avgX);
            sigmasY.add(avgY);
            sigmasH.add(avgH);
        }

        // Average all the calculated sigmas
        sigmaX_drift = sigmasX.stream().mapToDouble(Double::doubleValue).average().orElse(0);
        sigmaY_drift = sigmasY.stream().mapToDouble(Double::doubleValue).average().orElse(0);
        sigmaH_drift = sigmasH.stream().mapToDouble(Double::doubleValue).average().orElse(0);
    }

    private void updateKalmanFilter(double rawX, double rawY, double rawH)
    {
        // Get Limelight measurement
        limelight.updateRobotOrientation(rawH);
        LLResult llResult = limelight.getLatestResult();

        if (llResult == null || !llResult.isValid()) {
            return; // No update available
        }

        // Get Limelight position (FTC coordinates)
        Pose3D botPose_FTC = llResult.getBotpose_MT2();
        double ftcX_m = botPose_FTC.getPosition().x;
        double ftcY_m = botPose_FTC.getPosition().y;
        double ftcHeading_deg = botPose_FTC.getOrientation().getYaw(AngleUnit.DEGREES);

        // Convert to Pedro coordinates
        double ftcX_inches = ftcX_m * 39.3701;
        double ftcY_inches = ftcY_m * 39.3701;

        Pose limelightPose_FTC = new Pose(ftcX_inches, ftcY_inches, Math.toRadians(ftcHeading_deg), FTCCoordinates.INSTANCE);
        Pose limelightPose_Pedro = limelightPose_FTC.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        double limelightX = limelightPose_Pedro.getX();
        double limelightY = limelightPose_Pedro.getY();
        double limelightH = Math.toDegrees(limelightPose_Pedro.getHeading());

        // Kalman update step for each axis (1D simplified)
        // R = sensor noise variance (use measured limelight noise or default)
        double R_x = (sigmaX_limelight > 0 ? sigmaX_limelight * sigmaX_limelight : 2.0);
        double R_y = (sigmaY_limelight > 0 ? sigmaY_limelight * sigmaY_limelight : 2.0);
        double R_h = (sigmaH_limelight > 0 ? sigmaH_limelight * sigmaH_limelight : 1.0);

        // Update X
        double innovation_x = limelightX - (rawX - driftX);
        double S_x = P_x + R_x;
        double K_x = P_x / S_x;
        driftX = driftX + K_x * innovation_x;
        P_x = (1 - K_x) * P_x;

        // Update Y
        double innovation_y = limelightY - (rawY - driftY);
        double S_y = P_y + R_y;
        double K_y = P_y / S_y;
        driftY = driftY + K_y * innovation_y;
        P_y = (1 - K_y) * P_y;

        // Update Heading
        double innovation_h = limelightH - (rawH - driftH);
        double S_h = P_h + R_h;
        double K_h = P_h / S_h;
        driftH = driftH + K_h * innovation_h;
        P_h = (1 - K_h) * P_h;
    }

    private void displayTelemetry(double rawX, double rawY, double rawH,
                                   double correctedX, double correctedY, double correctedH)
    {
        telemetry.addData("═══ STATUS ═══", "");
        telemetry.addData("Collecting Data", collectingData ? "YES (" + driftSamples.size() + " samples)" : "NO");
        telemetry.addData("Filter Enabled", filterEnabled ? "YES" : "NO");
        telemetry.addData("Loop Time", String.format("%.1f ms", loopTime * 1000));

        telemetry.addData("", "");
        telemetry.addData("═══ NOISE PARAMETERS ═══", "");
        telemetry.addData("σ X drift", String.format("%.4f \"/√s", sigmaX_drift));
        telemetry.addData("σ Y drift", String.format("%.4f \"/√s", sigmaY_drift));
        telemetry.addData("σ H drift", String.format("%.4f °/√s", sigmaH_drift));
        telemetry.addData("σ X sensor", String.format("%.4f \"", sigmaX_sensor));
        telemetry.addData("σ Y sensor", String.format("%.4f \"", sigmaY_sensor));
        telemetry.addData("σ H sensor", String.format("%.4f °", sigmaH_sensor));

        telemetry.addData("", "");
        telemetry.addData("═══ DRIFT ESTIMATE ═══", "");
        telemetry.addData("Drift X", String.format("%.2f\" (P=%.3f)", driftX, P_x));
        telemetry.addData("Drift Y", String.format("%.2f\" (P=%.3f)", driftY, P_y));
        telemetry.addData("Drift H", String.format("%.2f° (P=%.3f)", driftH, P_h));

        telemetry.addData("", "");
        telemetry.addData("═══ POSITION ═══", "");
        telemetry.addData("Raw X", String.format("%.1f\"", rawX));
        telemetry.addData("Corrected X", String.format("%.1f\"", correctedX));
        telemetry.addData("Raw Y", String.format("%.1f\"", rawY));
        telemetry.addData("Corrected Y", String.format("%.1f\"", correctedY));
        telemetry.addData("Raw H", String.format("%.1f°", rawH));
        telemetry.addData("Corrected H", String.format("%.1f°", correctedH));

        telemetry.update();
    }

    // Helper class to store drift samples
    private static class DriftSample
    {
        double time;    // seconds
        double driftX;  // inches
        double driftY;  // inches
        double driftH;  // degrees

        DriftSample(double time, double driftX, double driftY, double driftH) {
            this.time = time;
            this.driftX = driftX;
            this.driftY = driftY;
            this.driftH = driftH;
        }
    }
}
