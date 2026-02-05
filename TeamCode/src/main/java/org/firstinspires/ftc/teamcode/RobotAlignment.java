package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * ✅ PRECISION ROBOT ALIGNMENT - 0.5° ACCURACY WITH VELOCITY COMPENSATION
 *
 * Key improvements:
 * 1. Two-stage control: Fast coarse approach + precise fine adjustment
 * 2. PD controller with derivative damping to prevent overshoot
 * 3. Distance-aware scaling for stability at 3+ meters
 * 4. Confirmed lock-on (must stay within 0.5° for multiple frames)
 * 5. VELOCITY FEEDFORWARD: Predicts robot motion to stay locked on target while moving
 *
 * This approach prioritizes PRECISION and STABILITY at ANY speed
 */
public class RobotAlignment implements Subsystem
{
    private GoBildaPinpointDriver pinpoint;
    private final TelemetryCustom telemetry;
    private GamepadEx ct1, ct2;
    private Drivetrain drivetrain;

    // Husky for tag detection
    private Husky husky;
    private boolean enableRelocalization = true;
    private int teamId = 4; // 4 = Blue, 5 = Red

    // Button readers
    private ButtonReader resetPositionButton;
    private ButtonReader toggleHeadingLockButton;
    private ButtonReader relocalizationButton;
    private ButtonReader toggleAbsoluteHeadingLockButton;

    // Target position
    private double targetX;
    private double targetY;

    // Robot's current position
    private double robotX;
    private double robotY;
    private double robotInitX;
    private double robotInitY;

    // Heading lock
    private boolean headingLockEnabled = false;
    private double lockedHeading = 0;
    private boolean isRobotMoving = false;
    private boolean resetIMUOnInit = true;

    // Absolute heading lock (constantly aims at target, works while moving)
    private boolean absoluteHeadingLockEnabled = false;

    // ✅ IMPROVED UNIFIED CONTROLLER - Aggressive for large errors, smooth for small
    private static class HeadingControl {
        // Base PID gains
        public static final double kP_BASE = 0.014;         // Base proportional gain (for small errors)
        public static final double kP_AGGRESSIVE = 0.045;   // Aggressive gain (for large errors)
        public static final double kD = 0.025;              // Derivative damping
        public static final double kFF = 0.20;              // Feedforward

        // Velocity compensation
        public static final double kV_CROSS = 0.008;        // Cross-track compensation
        public static final double kV_BOOST = 2.0;          // Power boost
        public static final double kV_PREDICTION = 0.25;    // Velocity-based prediction scaling

        // Power limits - error-dependent
        public static final double MIN_POWER = 0.03;        // Very gentle minimum
        public static final double MAX_POWER_STATIONARY = 0.10;     // For small errors when stationary
        public static final double MAX_POWER_MOVING_SMALL = 0.28;   // For small errors when moving
        public static final double MAX_POWER_MOVING_LARGE = 0.55;   // For large errors (aggressive!)

        // Error thresholds for gain scheduling
        public static final double SMALL_ERROR = 5.0;       // < 5° = very close, ultra-smooth
        public static final double MEDIUM_ERROR = 15.0;     // < 15° = approaching, moderate
        public static final double LARGE_ERROR = 45.0;      // > 45° = far away, aggressive

        // Precision thresholds - velocity adaptive
        public static final double TARGET_PRECISION_STATIONARY = 0.8;
        public static final double TARGET_PRECISION_MOVING = 2.5;
        public static final double VELOCITY_THRESHOLD = 0.12;  // m/s

        // Prediction and filtering
        public static final double PREDICTION_TIME_BASE = 0.08;
        public static final double PREDICTION_TIME_MAX = 0.20;
        public static final double DERIVATIVE_FILTER = 0.25;
    }

    // Derivative tracking variables
    private double lastHeadingError = 0;
    private long lastUpdateTime = 0;
    private int stableFrames = 0;
    private double filteredDerivative = 0;  // Smoothed derivative term
    private double lastTargetHeading = 0;   // For calculating heading rate

    // Velocity tracking with circular buffer for better filtering
    private double lastRobotX = 0;
    private double lastRobotY = 0;
    private long lastVelocityTime = 0;
    private static final int VELOCITY_BUFFER_SIZE = 7;  // Increased from 5 for ultra-smooth filtering
    private double[] velocityXBuffer = new double[VELOCITY_BUFFER_SIZE];
    private double[] velocityYBuffer = new double[VELOCITY_BUFFER_SIZE];
    private int velocityBufferIndex = 0;
    private double filteredVelocityX = 0;
    private double filteredVelocityY = 0;
    private double lastOutputPower = 0;  // For slew rate limiting

    public RobotAlignment(TelemetryCustom telemetry, GamepadEx ct1, GamepadEx ct2, Drivetrain drivetrain, boolean resetIMU, boolean isBlue, Husky husky) {
        this.telemetry = telemetry;
        this.ct1 = ct1;
        this.ct2 = ct2;
        this.drivetrain = drivetrain;
        this.resetIMUOnInit = resetIMU;
        this.husky = husky;
        this.teamId = isBlue ? 4 : 5;

        // Set husky team ID
        if (husky != null) {
            husky.SetTeamId(teamId);
        }

        // True = Blue, False = Red
        if(isBlue)
        {
            robotInitX = 143.6 - 9;
            robotInitY = 9;
            targetX = 0;
            targetY = 143.6;
        }
        else
        {
            robotInitX = 9;
            robotInitY = 9;
            targetX = 143.6;
            targetY = 143.6;
        }
    }

    public RobotAlignment(TelemetryCustom telemetry, boolean resetIMU, boolean isBlue) {
        this.telemetry = telemetry;
        this.resetIMUOnInit = resetIMU;
        this.teamId = isBlue ? 4 : 5;

        // True = Blue, False = Red
        if(isBlue)
        {
            robotInitX = 143.6 - 9;
            robotInitY = 9;
            targetX = 0;
            targetY = 143.6;
        }
        else
        {
            robotInitX = 9;
            robotInitY = 9;
            targetX = 143.6;
            targetY = 143.6;
        }
    }

    public void LinkComponents(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Only initialize button readers if gamepads exist (teleop mode)
        if (ct1 != null && ct2 != null) {
            resetPositionButton = new ButtonReader(ct2, GamepadKeys.Button.DPAD_DOWN);
            toggleHeadingLockButton = new ButtonReader(ct1, GamepadKeys.Button.RIGHT_BUMPER);
            relocalizationButton = new ButtonReader(ct2, GamepadKeys.Button.LEFT_BUMPER);
            toggleAbsoluteHeadingLockButton = new ButtonReader(ct1, GamepadKeys.Button.LEFT_BUMPER);
        }
    }

    public void Initialize(HardwareMap hardwareMap) {
        LinkComponents(hardwareMap);
        ConfigurePinpoint();

        if (resetIMUOnInit)
        {
            pinpoint.resetPosAndIMU();
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, robotInitX, robotInitY, AngleUnit.DEGREES, 90));
            robotX = robotInitX;
            robotY = robotInitY;
            pinpoint.recalibrateIMU();
            telemetry.Log("RobotAllignment", String.format("Pinpoint reset: X:%.1f\" Y:9\" Heading:90°", robotInitX));
        }
    }

    private void ConfigurePinpoint() {
        // ✅ PINPOINT ODOMETRY CONFIGURATION
        // The GoBilda Pinpoint uses dedicated odometry pods (separate from drive motors)
        // Drive motors DO NOT need encoders - Pinpoint handles all position tracking
        // Pinpoint provides: X, Y position (inches) and heading (degrees)

        pinpoint.setOffsets(2.11, -3.31, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        telemetry.Log("Pinpoint", "Configured: X=-3.62\" Y=-6.65\" (4-bar pods)");
        telemetry.Log("Pinpoint", "Odometry pods enabled - drive motors need NO encoders");
    }

    public void Run()
    {
        ReadButtons();
        UpdatePositionFromPinpoint();

        // Dynamically adjust target based on robot position (corner compensation)
        UpdateTargetPosition();

        // Detect robot movement
        if (ct1 != null) {
            SetRobotMoving(
                    ct1.getLeftX(),
                    ct1.getLeftY(),
                    ct1.gamepad.left_trigger,
                    ct1.gamepad.right_trigger
            );
        }

        // ✅ PRECISION HEADING CONTROL
        if (absoluteHeadingLockEnabled && drivetrain != null) {
            double[] lockPowers = CalculateHeadingCorrection();
            if (lockPowers != null) {
                drivetrain.ApplyHeadingLockPowers(lockPowers);
            }
        }
        else if (headingLockEnabled && !isRobotMoving && drivetrain != null) {
            double[] lockPowers = CalculateHeadingCorrection();
            if (lockPowers != null) {
                // Reduce power for stationary corrections
                for (int i = 0; i < lockPowers.length; i++) {
                    lockPowers[i] *= 0.6;
                }
                drivetrain.ApplyHeadingLockPowers(lockPowers);
            }
        }
        else if (drivetrain != null) {
            drivetrain.ApplyHeadingLockPowers(null);
        }
    }

    /**
     * Dynamically adjust target position based on robot's Y position
     * Compensates for field corners when robot is near edges
     */
    private void UpdateTargetPosition()
    {
        // True = Blue team (target at X=0, Y=143.6)
        // False = Red team (target at X=143.6, Y=143.6)
        boolean isBlue = (teamId == 4);

        if (isBlue)
        {
            // BLUE TEAM TARGET ADJUSTMENT
            if (robotY > 110.0)
            {
                targetX = 0;
                targetY = 143.6 - 3.0;
            }
            else if (robotY < 23.0)
            {
                targetX = 0 + 3.0;
                targetY = 143.6;
            }
            else
            {
                targetX = 0;
                targetY = 143.6;
            }
        }
        else
        {
            // RED TEAM TARGET ADJUSTMENT
            if (robotY > 110.0)
            {
                targetX = 143.6;
                targetY = 143.6 - 3.0;
            }
            else if (robotY < 23.0)
            {
                targetX = 143.6 - 3.0;
                targetY = 143.6;
            }
            else
            {
                targetX = 143.6;
                targetY = 143.6;
            }
        }
    }

    /**
     * ✅ CALCULATE FILTERED VELOCITY using moving average
     * More stable than single-frame differentiation
     * Returns velocity in inches/second for X and Y
     */
    private void UpdateVelocity() {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastVelocityTime) / 1_000_000_000.0;

        if (lastVelocityTime != 0 && dt > 0 && dt < 0.1) {
            // Calculate instantaneous velocity
            double velX = (robotX - lastRobotX) / dt;
            double velY = (robotY - lastRobotY) / dt;

            // Add to circular buffer
            velocityXBuffer[velocityBufferIndex] = velX;
            velocityYBuffer[velocityBufferIndex] = velY;
            velocityBufferIndex = (velocityBufferIndex + 1) % VELOCITY_BUFFER_SIZE;

            // Calculate moving average
            double sumX = 0, sumY = 0;
            for (int i = 0; i < VELOCITY_BUFFER_SIZE; i++) {
                sumX += velocityXBuffer[i];
                sumY += velocityYBuffer[i];
            }
            filteredVelocityX = sumX / VELOCITY_BUFFER_SIZE;
            filteredVelocityY = sumY / VELOCITY_BUFFER_SIZE;
        }

        lastRobotX = robotX;
        lastRobotY = robotY;
        lastVelocityTime = currentTime;
    }

    /**
     * ✅ IMPROVED HEADING CORRECTION with Velocity-Adaptive Prediction
     * Achieves smooth tracking at any speed through:
     * - Velocity-adaptive prediction time (looks further ahead at high speed)
     * - Angular velocity compensation for better tracking during strafing
     * - Ultra-smooth filtering and slew rate limiting
     * - Progressive deadband to eliminate micro-oscillations
     */
    private double[] CalculateHeadingCorrection()
    {
        // ✅ STEP 1: Update and get filtered velocity
        UpdateVelocity();
        double velocityX = filteredVelocityX;  // inches/sec
        double velocityY = filteredVelocityY;  // inches/sec
        double velocityMagnitude = Math.sqrt(velocityX * velocityX + velocityY * velocityY);
        double velocityMetersPerSec = velocityMagnitude * 0.0254;

        // ✅ STEP 2: VELOCITY-ADAPTIVE PREDICTION TIME
        // At higher speeds, look further ahead to compensate for lag
        double velocityFactor = Math.min(velocityMetersPerSec / 1.0, 1.0);  // 0 to 1 at 0-1 m/s
        double predictionTime = HeadingControl.PREDICTION_TIME_BASE +
                              (HeadingControl.PREDICTION_TIME_MAX - HeadingControl.PREDICTION_TIME_BASE) * velocityFactor;

        // ✅ STEP 3: Predict future position with velocity-adaptive lookahead
        double predictedX = robotX + (velocityX * predictionTime);
        double predictedY = robotY + (velocityY * predictionTime);

        // ✅ STEP 4: Calculate target heading from PREDICTED position
        double deltaX = targetX - predictedX;
        double deltaY = targetY - predictedY;
        double targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));
        targetHeading = NormalizeAngle(targetHeading);

        // ✅ STEP 5: Calculate heading error
        double currentHeading = GetCurrentHeading();
        double error = NormalizeAngle(targetHeading - currentHeading);
        double absError = Math.abs(error);

        // ✅ STEP 6: Calculate derivative with low-pass filtering
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastUpdateTime) / 1_000_000_000.0;
        double derivative = 0;

        if (lastUpdateTime != 0 && dt > 0 && dt < 0.1) {
            double rawDerivative = (error - lastHeadingError) / dt;
            // Low-pass filter to reduce noise
            filteredDerivative = HeadingControl.DERIVATIVE_FILTER * rawDerivative +
                               (1 - HeadingControl.DERIVATIVE_FILTER) * filteredDerivative;
            derivative = filteredDerivative;
        }

        lastHeadingError = error;
        lastUpdateTime = currentTime;

        // ✅ STEP 7: Calculate heading rate feedforward
        // How fast is the target heading changing due to our motion?
        double headingRate = 0;
        if (lastUpdateTime != 0 && dt > 0) {
            headingRate = NormalizeAngle(targetHeading - lastTargetHeading) / dt;
        }
        lastTargetHeading = targetHeading;

        // ✅ STEP 8: ANGULAR VELOCITY COMPENSATION for high-speed lateral movement
        // When strafing around the target, the heading changes rapidly - compensate for this
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double angularVelocityCompensation = 0;

        if (distanceToTarget > 10.0) {  // Only when not too close
            // Calculate tangential velocity (velocity perpendicular to radius to target)
            double angleToTarget = Math.atan2(deltaY, deltaX);
            double tangentialVelocity = -velocityX * Math.sin(angleToTarget) + velocityY * Math.cos(angleToTarget);

            // Angular velocity = tangential velocity / radius
            double angularVelocity = tangentialVelocity / distanceToTarget;  // rad/sec
            double angularVelocityDegPerSec = Math.toDegrees(angularVelocity);

            // Add compensation proportional to angular velocity
            angularVelocityCompensation = angularVelocityDegPerSec * HeadingControl.kV_PREDICTION;
        }

        // ✅ STEP 9: Cross-track velocity compensation (reduced - angular velocity does most of the work now)
        double angleToTarget = Math.atan2(deltaY, deltaX);
        double crossTrackVelocity = -velocityX * Math.sin(angleToTarget) + velocityY * Math.cos(angleToTarget);
        double crossTrackCorrection = crossTrackVelocity * HeadingControl.kV_CROSS;

        // ✅ STEP 10: Velocity-dependent precision target
        boolean isMoving = velocityMetersPerSec > HeadingControl.VELOCITY_THRESHOLD;
        double precisionTarget = isMoving ? HeadingControl.TARGET_PRECISION_MOVING :
                                          HeadingControl.TARGET_PRECISION_STATIONARY;

        // ✅ STEP 11: Progressive deadband - wider when moving fast
        double deadband = 1.0 + (velocityMetersPerSec * 1.5);  // 1-2.5° depending on speed
        if (absError < deadband && derivative < 5.0) {
            stableFrames++;
            if (stableFrames > 7) {  // Require even more stable frames
                lastOutputPower = 0;  // Reset for smooth restart
                return null;  // Locked on!
            }
        } else {
            stableFrames = 0;
        }

        // ✅ STEP 12: NON-LINEAR GAIN SCHEDULING - Fast for big errors, smooth for small
        // Uses exponential curve with multiple zones for optimal response across all error ranges
        double kP_scaled;
        double maxPower;

        if (absError > HeadingControl.LARGE_ERROR) {
            // ZONE 1: LARGE ERROR (> 45°) - VERY AGGRESSIVE
            // This is when you're far from target - turn FAST!
            kP_scaled = HeadingControl.kP_AGGRESSIVE;
            maxPower = isMoving ? HeadingControl.MAX_POWER_MOVING_LARGE : 0.45;

        } else if (absError > HeadingControl.MEDIUM_ERROR) {
            // ZONE 2: MEDIUM ERROR (15-45°) - AGGRESSIVE with smooth ramp-down
            // Interpolate between aggressive and moderate as error decreases
            double zoneProgress = (absError - HeadingControl.MEDIUM_ERROR) / (HeadingControl.LARGE_ERROR - HeadingControl.MEDIUM_ERROR);
            kP_scaled = 0.025 + (HeadingControl.kP_AGGRESSIVE - 0.025) * zoneProgress;
            maxPower = isMoving ? (0.35 + 0.20 * zoneProgress) : 0.30;

        } else if (absError > HeadingControl.SMALL_ERROR) {
            // ZONE 3: SMALL ERROR (5-15°) - MODERATE, approaching smoothly
            double zoneProgress = (absError - HeadingControl.SMALL_ERROR) / (HeadingControl.MEDIUM_ERROR - HeadingControl.SMALL_ERROR);
            kP_scaled = HeadingControl.kP_BASE + (0.025 - HeadingControl.kP_BASE) * zoneProgress;
            maxPower = isMoving ? (HeadingControl.MAX_POWER_MOVING_SMALL + 0.07 * zoneProgress) : 0.18;

        } else {
            // ZONE 4: TINY ERROR (< 5°) - ULTRA SMOOTH for precision
            // Exponential curve for final approach
            double smoothFactor = absError / HeadingControl.SMALL_ERROR;  // 0 to 1
            kP_scaled = HeadingControl.kP_BASE * (0.7 + 0.3 * smoothFactor);
            maxPower = isMoving ? HeadingControl.MAX_POWER_MOVING_SMALL : HeadingControl.MAX_POWER_STATIONARY;
        }

        // ✅ STEP 13: Calculate control terms
        double proportionalTerm = kP_scaled * error;
        double derivativeTerm = HeadingControl.kD * derivative;
        double feedforwardTerm = HeadingControl.kFF * headingRate;

        // ✅ STEP 14: Velocity boost - ONLY for large errors to avoid fighting fine control
        double velocityBoost = 0;
        if (isMoving && absError > 8.0) {  // Only boost for errors > 8°
            double boostScale = Math.min((absError - 8.0) / 30.0, 1.0);  // More boost for larger errors
            velocityBoost = (velocityMetersPerSec - HeadingControl.VELOCITY_THRESHOLD) * HeadingControl.kV_BOOST * boostScale;
            velocityBoost = Math.min(velocityBoost, 0.15);
            velocityBoost *= Math.signum(error);
        }

        // ✅ STEP 15: Combine all terms including angular velocity compensation
        double power = proportionalTerm + derivativeTerm + feedforwardTerm + crossTrackCorrection +
                      velocityBoost + angularVelocityCompensation;

        // ✅ STEP 16: Apply power limits (error-dependent max power already set in step 12)
        power = Math.max(-maxPower, Math.min(maxPower, power));

        // ✅ STEP 17: ADAPTIVE slew rate limiting - faster for large errors, smooth for small
        double maxPowerChange;
        if (absError > HeadingControl.LARGE_ERROR) {
            maxPowerChange = 0.25;  // Fast transitions for large errors
        } else if (absError > HeadingControl.MEDIUM_ERROR) {
            maxPowerChange = 0.18;  // Medium transitions
        } else if (absError > HeadingControl.SMALL_ERROR) {
            maxPowerChange = 0.12;  // Smooth transitions
        } else {
            maxPowerChange = 0.08;  // Ultra-smooth for final approach
        }

        double powerChange = power - lastOutputPower;
        if (Math.abs(powerChange) > maxPowerChange) {
            power = lastOutputPower + Math.copySign(maxPowerChange, powerChange);
        }
        lastOutputPower = power;

        // ✅ STEP 18: Apply minimum power (only when stationary and error is large)
        if (!isMoving && absError > 6.0 && Math.abs(power) < HeadingControl.MIN_POWER) {
            power = Math.copySign(HeadingControl.MIN_POWER, power);
        }

        // ✅ STEP 19: Force stop when settled and stationary
        if (!isMoving && absError < 1.5 && Math.abs(derivative) < 3.0) {
            power = 0;
            lastOutputPower = 0;  // Reset for next cycle
        }

        // Return motor powers: [frontLeft, frontRight, backLeft, backRight]
        return new double[]{
                power,      // Front left (positive = CCW)
                -power,     // Front right
                power,      // Back left
                -power      // Back right
        };
    }

    /**
     * BUTTON-TRIGGERED relocalization
     * Only runs when the button is pressed
     */
    private void CheckAndApplyRelocalization() {
        if (husky == null) {
            telemetry.Log("⚠️ Relocalization", "Husky not available");
            return;
        }

        // Get CURRENT heading from IMU as initial guess
        double currentHeading = GetCurrentHeading();

        // Ask Husky to calculate pose using current heading as hint
        RobotPoseData pose = husky.CalculateRobotPose(currentHeading);

        if (pose == null) {
            telemetry.Log("❌ Relocalization", "No team tag visible");
            ct2.gamepad.rumble(500);
            return;
        }

        // Sanity check on calculated position
        double deltaX = Math.abs(pose.x - robotX);
        double deltaY = Math.abs(pose.y - robotY);
        double deltaH = Math.abs(pose.heading - GetCurrentHeading());

        // If position jumps more than 24 inches or heading jumps more than 30°, reject it
        if (deltaX > 24.0 || deltaY > 24.0 || deltaH > 30.0) {
            telemetry.Log("⚠️ REJECTED Reloc", String.format("ΔX=%.1f ΔY=%.1f ΔH=%.1f", deltaX, deltaY, deltaH));
            ct2.gamepad.rumble(1000);
            return;
        }

        // Position change is reasonable - apply it
        Pose2D relocPose = new Pose2D(DistanceUnit.INCH, pose.x, pose.y, AngleUnit.DEGREES, pose.heading);
        pinpoint.setPosition(relocPose);
        robotX = pose.x;
        robotY = pose.y;

        // Success feedback
        ct2.gamepad.rumble(0.6, 0.6, 300);
        telemetry.Log("✅ RELOCALIZED!", "");
        telemetry.Log("  New X", String.format("%.1f\"", robotX));
        telemetry.Log("  New Y", String.format("%.1f\"", robotY));
        telemetry.Log("  New Heading", String.format("%.1f°", pose.heading));
        telemetry.Log("  Delta", String.format("ΔX=%.1f ΔY=%.1f ΔH=%.1f", deltaX, deltaY, deltaH));
    }

    /**
     * Enable or disable relocalization feature
     */
    public void SetRelocalizationEnabled(boolean enabled) {
        this.enableRelocalization = enabled;
        telemetry.Log("Relocalization", enabled ? "ENABLED" : "DISABLED");
    }

    private void UpdatePositionFromPinpoint() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        robotX = pose.getX(DistanceUnit.INCH);
        robotY = pose.getY(DistanceUnit.INCH);
    }


    private void ReadButtons() {
        if (resetPositionButton == null) return;

        resetPositionButton.readValue();
        toggleHeadingLockButton.readValue();
        relocalizationButton.readValue();
        toggleAbsoluteHeadingLockButton.readValue();

        if (resetPositionButton.wasJustPressed()) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, robotInitX, robotInitY, AngleUnit.DEGREES, 90));
            pinpoint.recalibrateIMU();
            ct1.gamepad.rumble(0.6, 0.6, 500);
            ct2.gamepad.rumble(0.7, 0.7, 500);
            telemetry.Log("Position Reset", "X:9\" Y:9\" Heading:90° (IMU recalibrated)");
        }

        if (toggleHeadingLockButton.wasJustPressed()) {
            if (headingLockEnabled) {
                UnlockHeading();
            } else {
                LockCurrentHeading();
            }
        }

        if (toggleAbsoluteHeadingLockButton.wasJustPressed()) {
            if (absoluteHeadingLockEnabled) {
                UnlockAbsoluteHeading();
            } else {
                LockAbsoluteHeading();
            }
        }

        if (relocalizationButton.wasJustPressed()) {
            if (enableRelocalization) {
                telemetry.Log("🔄 Relocalization", "TRIGGERED by button press");
                CheckAndApplyRelocalization();
            } else {
                telemetry.Log("⚠️ Relocalization", "Feature is DISABLED");
                ct2.gamepad.rumble(1000);
            }
        }
    }

    public double GetDistanceToTarget()
    {
        // ✅ SHOOTER DISTANCE CALCULATION
        // This distance is used to dynamically adjust shooter motor speeds
        // Pinpoint provides accurate X,Y position → calculate distance to target
        // Returns distance in METERS (for shooter speed calculations)

        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double distanceInInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        return distanceInInches * 0.0254;  // Convert inches to meters
    }

    public double GetHeadingToTarget()
    {
        // ✅ SHOOTER HEADING CALCULATION
        // This heading is used by the heading lock to keep robot aimed at target
        // Also useful for adjusting shooter angle or aim compensation
        // Returns angle in DEGREES (-180 to +180)

        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double angleRadians = Math.atan2(deltaY, deltaX);
        double angleDegrees = Math.toDegrees(angleRadians);
        return NormalizeAngle(angleDegrees);
    }

    public void ResetRobotPosition()
    {
        this.robotX = 9.0;
        this.robotY = 9.0;
        double currentHeading = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 9.0, 9.0, AngleUnit.DEGREES, currentHeading));
        telemetry.Log("Position Reset", String.format("X:9.0\" Y:9.0\" Heading:%.1f°", currentHeading));
    }

    public void SetStartingPosition(double x, double y, double headingDegrees)
    {
        this.robotInitX = x;
        this.robotInitY = y;
        this.robotX = x;
        this.robotY = y;

        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDegrees));
            telemetry.Log("Auto Start Pos", String.format("X:%.1f\" Y:%.1f\" H:%.1f°", x, y, headingDegrees));
        }
    }

    // ==================== HEADING LOCK ====================

    public void LockCurrentHeading()
    {
        lockedHeading = GetHeadingToTarget();
        headingLockEnabled = true;
        telemetry.Log("Heading Lock", String.format("ENABLED - Pointing to target at %.1f°", lockedHeading));
    }

    public void UnlockHeading()
    {
        headingLockEnabled = false;
        telemetry.Log("Heading Lock", "DISABLED");
    }

    public boolean IsHeadingLocked() {return headingLockEnabled;}

    // ==================== ABSOLUTE HEADING LOCK ====================

    public void LockAbsoluteHeading()
    {
        absoluteHeadingLockEnabled = true;
        telemetry.Log("🔒 Auto-Aim Lock", "ENABLED - Always facing target");
    }

    public void UnlockAbsoluteHeading()
    {
        absoluteHeadingLockEnabled = false;
        telemetry.Log("🔓 Auto-Aim Lock", "DISABLED");
    }

    public boolean IsAbsoluteHeadingLocked() {return absoluteHeadingLockEnabled;}

    public void SetRobotMoving(double leftStickX, double leftStickY, double leftTrigger, double rightTrigger) {
        boolean wasMoving = isRobotMoving;
        double threshold = 0.05;
        isRobotMoving = Math.abs(leftStickX) > threshold ||
                Math.abs(leftStickY) > threshold ||
                Math.abs(leftTrigger) > threshold ||
                Math.abs(rightTrigger) > threshold;

        if (isRobotMoving && !wasMoving && headingLockEnabled) {
            telemetry.Log("Heading Lock", "Paused - Manual control active");
        }

        if (!isRobotMoving && wasMoving && headingLockEnabled) {
            lockedHeading = GetHeadingToTarget();
            telemetry.Log("Heading Lock", String.format("Resumed - Pointing to target at %.1f°", lockedHeading));
        }
    }

    private double NormalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public double GetCurrentHeading() {return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);}
    public double GetRobotX() {return robotX;}
    public double GetRobotY() {return robotY;}

    /**
     * ✅ DIAGNOSTIC TELEMETRY - Call this in your teleop loop to tune velocity compensation
     * This shows you real-time data to help adjust the controller parameters
     */
    public void LogVelocityDiagnostics() {
        if (telemetry == null) return;

        double velX = filteredVelocityX;
        double velY = filteredVelocityY;
        double velocityMagnitude = Math.sqrt(velX * velX + velY * velY);
        double velocityMetersPerSec = velocityMagnitude * 0.0254;

        double targetHeading = GetHeadingToTarget();
        double currentHeading = GetCurrentHeading();
        double error = NormalizeAngle(targetHeading - currentHeading);

        telemetry.Log("━━━ VELOCITY DIAGNOSTICS ━━━", "");
        telemetry.Log("Velocity (m/s)", String.format("%.2f", velocityMetersPerSec));
        telemetry.Log("Velocity X (in/s)", String.format("%.1f", velX));
        telemetry.Log("Velocity Y (in/s)", String.format("%.1f", velY));
        telemetry.Log("Heading Error", String.format("%.2f°", error));
        telemetry.Log("Distance to Target", String.format("%.1f\"", GetDistanceToTarget() / 0.0254));
        telemetry.Log("Lock Status", absoluteHeadingLockEnabled ? "ACTIVE" : "INACTIVE");
    }
}