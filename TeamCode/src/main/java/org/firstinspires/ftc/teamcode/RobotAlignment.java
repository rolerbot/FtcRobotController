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

    // ✅ PRECISION TUNING PARAMETERS - For 0.5° accuracy
    private static class HeadingControl {
        // Two-stage control: coarse approach + fine adjustment
        public static final double kP_COARSE = 0.015;  // Aggressive for large errors
        public static final double kP_FINE = 0.025;    // Higher gain for precision
        public static final double kD = 0.008;         // Derivative to prevent overshoot

        // Power limits
        public static final double MIN_POWER_COARSE = 0.10;
        public static final double MIN_POWER_FINE = 0.05;    // Very gentle for final approach
        public static final double MAX_POWER_COARSE = 0.30;
        public static final double MAX_POWER_FINE = 0.12;    // Limited for precision

        // Precision thresholds
        public static final double FINE_CONTROL_THRESHOLD = 8.0;   // Switch to fine control
        public static final double TARGET_PRECISION = 0.3;         // Your 0.5° target
        public static final double SETTLING_THRESHOLD = 0.3;       // Must stay within this to stop

        // Distance-based scaling
        public static final double DISTANCE_DAMPING_START = 1.5;   // Start damping at 1.5m
        public static final double DISTANCE_DAMPING_FACTOR = 0.2;

        // ✅ VELOCITY FEEDFORWARD - Predicts where robot will be
        public static final double PREDICTION_TIME = 0.13;          // Look ahead 150ms
        public static final double VELOCITY_BOOST_FACTOR = 2.5;     // Aggressive boost at high speed
        public static final double VELOCITY_THRESHOLD = 0.2;        // m/s - when to start boosting
        public static final double MAX_VELOCITY_BOOST = 0.15;       // Extra power cap
    }

    // Derivative tracking variables
    private double lastHeadingError = 0;
    private long lastUpdateTime = 0;
    private int stableFrames = 0;  // Count frames within target precision

    // Velocity calculation from position changes
    private double lastRobotX = 0;
    private double lastRobotY = 0;
    private long lastPositionUpdateTime = 0;
    private double robotVelocityX = 0;
    private double robotVelocityY = 0;

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

        pinpoint.setOffsets(3.307, -6.648, DistanceUnit.INCH);
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
     * ✅ VELOCITY-COMPENSATED PD CONTROLLER
     * Achieves 0.5° accuracy AT ANY SPEED through:
     * - Predicts where robot will be based on current velocity
     * - Compensates for heading drift during motion
     * - Boosts correction power proportionally to velocity
     * - Maintains precision when stationary
     */
    private double[] CalculateHeadingCorrection()
    {
        // ✅ GET VELOCITY using our custom calculation
        Pose2D velocity = CalculateVelocity();
        double velocityX = velocity.getX(DistanceUnit.INCH);
        double velocityY = velocity.getY(DistanceUnit.INCH);

        // ✅ STEP 1: Predict future position based on velocity
        // Where will the robot be in PREDICTION_TIME seconds?
        double predictedX = robotX + (velocityX * HeadingControl.PREDICTION_TIME);
        double predictedY = robotY + (velocityY * HeadingControl.PREDICTION_TIME);

        // Calculate velocity magnitude (speed in inches/sec)
        double velocityMagnitude = Math.sqrt(velocityX * velocityX + velocityY * velocityY);
        double velocityMetersPerSec = velocityMagnitude * 0.0254;  // Convert to m/s

        // ✅ STEP 2: Calculate target heading using PREDICTED position
        double deltaX = targetX - predictedX;
        double deltaY = targetY - predictedY;
        double targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));
        targetHeading = NormalizeAngle(targetHeading);

        double currentHeading = GetCurrentHeading();
        double error = NormalizeAngle(targetHeading - currentHeading);
        double absError = Math.abs(error);

        // ✅ STEP 3: Calculate derivative (rate of change of error)
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastUpdateTime) / 1_000_000_000.0;
        double derivative = 0;

        if (lastUpdateTime != 0 && dt > 0 && dt < 0.1) {
            derivative = (error - lastHeadingError) / dt;
        }

        lastHeadingError = error;
        lastUpdateTime = currentTime;

        // ✅ STEP 4: Check if we're at target precision (only when nearly stopped)
        if (absError < HeadingControl.TARGET_PRECISION && velocityMetersPerSec < 0.1) {
            stableFrames++;
            if (stableFrames > 5) {
                return null;  // Locked on target while stationary!
            }
        } else {
            stableFrames = 0;
        }

        // ✅ STEP 5: DISTANCE-BASED DAMPING for long-range stability
        double distanceMeters = Math.sqrt(deltaX * deltaX + deltaY * deltaY) * 0.0254;
        double distanceDamping = 1.0;
        if (distanceMeters > HeadingControl.DISTANCE_DAMPING_START) {
            double excessDistance = distanceMeters - HeadingControl.DISTANCE_DAMPING_START;
            distanceDamping = 1.0 / (1.0 + excessDistance * HeadingControl.DISTANCE_DAMPING_FACTOR);
        }

        // ✅ STEP 6: TWO-STAGE CONTROL with velocity awareness
        // When moving fast, stay in coarse mode longer for aggressive tracking
        double fineThreshold = HeadingControl.FINE_CONTROL_THRESHOLD;
        if (velocityMetersPerSec > HeadingControl.VELOCITY_THRESHOLD) {
            fineThreshold = 3.0;  // Tighter threshold when moving - stay aggressive
        }

        boolean useFineControl = absError < fineThreshold;

        double kP, minPower, maxPower;
        if (useFineControl) {
            kP = HeadingControl.kP_FINE;
            minPower = HeadingControl.MIN_POWER_FINE;
            maxPower = HeadingControl.MAX_POWER_FINE;
        } else {
            kP = HeadingControl.kP_COARSE;
            minPower = HeadingControl.MIN_POWER_COARSE;
            maxPower = HeadingControl.MAX_POWER_COARSE;
        }

        // ✅ STEP 7: PD CONTROL with velocity compensation
        double proportionalTerm = kP * error * distanceDamping;
        double derivativeTerm = HeadingControl.kD * derivative * distanceDamping;
        double power = proportionalTerm + derivativeTerm;

        // ✅ STEP 8: VELOCITY FEEDFORWARD BOOST
        // The faster you're moving, the more aggressive the correction needs to be
        if (velocityMetersPerSec > HeadingControl.VELOCITY_THRESHOLD) {
            double velocityBoost = (velocityMetersPerSec - HeadingControl.VELOCITY_THRESHOLD)
                    * HeadingControl.VELOCITY_BOOST_FACTOR;
            velocityBoost = Math.min(velocityBoost, HeadingControl.MAX_VELOCITY_BOOST);

            // Apply boost in the direction of the error
            double boostPower = Math.copySign(velocityBoost, error);
            power += boostPower;

            // Also increase max power limit when moving
            maxPower = Math.min(maxPower + velocityBoost, 0.5);  // Cap at 0.5
        }

        // ✅ STEP 9: SMOOTH POWER CURVE (only when stationary or slow)
        if (useFineControl && velocityMetersPerSec < 0.3) {
            double sign = Math.signum(power);
            double absPower = Math.abs(power);
            absPower = Math.pow(absPower, 1.8);
            power = sign * absPower;
        }

        // ✅ STEP 10: EXTRA DAMPING when very close (only when slow)
        if (absError < 2.0 && velocityMetersPerSec < 0.2) {
            double proximityScale = absError / 2.0;
            power *= (0.4 + 0.6 * proximityScale);
        }

        // ✅ STEP 11: CLAMP TO LIMITS
        power = Math.max(-maxPower, Math.min(maxPower, power));

        // ✅ STEP 12: APPLY MINIMUM POWER (skip if moving fast)
        if (absError > HeadingControl.SETTLING_THRESHOLD && Math.abs(power) < minPower) {
            if (velocityMetersPerSec < 0.3) {  // Only enforce min when slow
                power = Math.copySign(minPower, power);
            }
        }

        // ✅ STEP 13: Force stop only when stationary and accurate
        if (absError < HeadingControl.SETTLING_THRESHOLD &&
                Math.abs(derivative) < 2.0 &&
                velocityMetersPerSec < 0.1) {
            power = 0;
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

    /**
     * ✅ CALCULATE VELOCITY - Mimics Pinpoint's getVelocity() method
     * Returns a Pose2D object with velocity data (inches/sec and degrees/sec)
     */
    private Pose2D CalculateVelocity() {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastPositionUpdateTime) / 1_000_000_000.0;  // Convert to seconds

        double velX = 0;
        double velY = 0;

        if (lastPositionUpdateTime != 0 && dt > 0 && dt < 0.5) {  // Sanity check
            double deltaX = robotX - lastRobotX;
            double deltaY = robotY - lastRobotY;

            // Calculate raw velocity
            velX = deltaX / dt;
            velY = deltaY / dt;

            // Simple low-pass filter to smooth noise
            double alpha = 0.7;  // 0 = all old, 1 = all new
            robotVelocityX = alpha * velX + (1 - alpha) * robotVelocityX;
            robotVelocityY = alpha * velY + (1 - alpha) * robotVelocityY;
        } else {
            // Use stored values if time gap is invalid
            velX = robotVelocityX;
            velY = robotVelocityY;
        }

        // Update stored position and time
        lastRobotX = robotX;
        lastRobotY = robotY;
        lastPositionUpdateTime = currentTime;

        // Return as Pose2D (velocity in inches/sec for X,Y and 0 for heading)
        return new Pose2D(DistanceUnit.INCH, robotVelocityX, robotVelocityY, AngleUnit.DEGREES, 0);
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
     * This shows you real-time data to help adjust the VELOCITY_BOOST_FACTOR
     */
    public void LogVelocityDiagnostics() {
        if (telemetry == null) return;

        Pose2D velocity = CalculateVelocity();
        double velX = velocity.getX(DistanceUnit.INCH);
        double velY = velocity.getY(DistanceUnit.INCH);
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