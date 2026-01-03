package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RobotAllignment implements Subsystem
{
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;  // Odometry computer for position tracking
    private final TelemetryCustom telemetry;
    private GamepadEx ct1, ct2;
    private Drivetrain drivetrain;  // Reference to drivetrain for applying heading lock

    // Button readers
    private ButtonReader resetPositionButton;
    private ButtonReader toggleHeadingLockButton;

    // PID coefficients for rotation - using Pedro Pathing's tuned heading PID
    private final double kP;

    // Target position for distance calculation
    // (in INCHES - Pedro Pathing units) X:
    private double targetX = 1;
    private double targetY = 142.056;

    // Robot's current position on field (in INCHES - Pedro Pathing units)
    private double robotX = 9.000;  // Default starting X position in inches
    private double robotY = 9.000;    // Default starting Y position in inches

    // Heading lock feature - maintains heading continuously
    private boolean headingLockEnabled = false;  // Set to true to enable continuous heading correction
    private double lockedHeading = 0;
    private boolean isRobotMoving = false;  // Track if robot is currently moving

    private boolean resetIMUOnInit = true;

    public RobotAllignment(TelemetryCustom telemetry, GamepadEx ct1, GamepadEx ct2, Drivetrain drivetrain, boolean resetIMU) {
        this.telemetry = telemetry;
        this.ct1 = ct1;
        this.ct2 = ct2;
        this.drivetrain = drivetrain;
        this.resetIMUOnInit = resetIMU;

        // Use Pedro Pathing's tuned heading PID coefficient
        this.kP = 0.65;   // Proportional gain
    }

    public RobotAllignment(TelemetryCustom telemetry, boolean resetIMU) {
        this.telemetry = telemetry;
        this.resetIMUOnInit = resetIMU;

        // Use Pedro Pathing's tuned heading PID coefficient
        this.kP = 0.65;   // Proportional gain
    }

    public void LinkComponents(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Only initialize button readers if gamepads exist (teleop mode)
        if (ct1 != null && ct2 != null) {
            resetPositionButton = new ButtonReader(ct2, GamepadKeys.Button.DPAD_DOWN);
            toggleHeadingLockButton = new ButtonReader(ct2, GamepadKeys.Button.DPAD_LEFT);
        }
    }

    public void Initialize(HardwareMap hardwareMap) {
        LinkComponents(hardwareMap);

        // Initialize IMU (backup - Pinpoint has its own IMU)
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // ===== CRITICAL: Configure Pinpoint Odometry Computer =====
        ConfigurePinpoint();

        // Optionally reset position and IMU (useful for autonomous start)
        if (resetIMUOnInit) {
            pinpoint.resetPosAndIMU();  // Resets position to (0,0,0) and recalibrates IMU
            imu.resetYaw();  // Also reset backup IMU
            telemetry.Log("ShooterDistance", "Pinpoint & IMU reset to 0°");
        } else {
            telemetry.Log("ShooterDistance", "Pinpoint preserving heading: " + GetCurrentHeading() + "°");
        }

        telemetry.Log("ShooterDistance", "Init: PID_kP=" + kP);
    }

    /**
     * Configure the Pinpoint odometry computer
     * Uses the same configuration as Pedro Pathing for consistency
     */
    private void ConfigurePinpoint() {
        // Set odometry pod positions relative to robot center
        // These values MATCH your Constants.java configuration for Pedro Pathing
        // Forward pod Y offset: -6.65" (backward of center)
        // Strafe pod X offset: -3.62" (right of center)
        pinpoint.setOffsets(-3.62, -6.65, DistanceUnit.INCH);

        // Set the type of odometry pods (goBILDA 4-bar pods)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions (both FORWARD, matching Pedro Pathing)
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        telemetry.Log("Pinpoint", "Configured: X=-3.62\" Y=-6.65\" (4-bar pods)");
    }

    /**
     * Run method called periodically - handles button inputs and position tracking
     */
    public void Run()
    {
        ReadButtons();  // Read buttons FIRST to handle position resets
        UpdatePositionFromPinpoint();  // Then read current position from odometry

        // Automatically detect robot movement from drivetrain inputs (if gamepads exist)
        if (ct1 != null) {
            SetRobotMoving(
                ct1.getLeftX(),
                ct1.getLeftY(),
                ct1.gamepad.left_trigger,   // Left trigger (rotation)
                ct1.gamepad.right_trigger   // Right trigger (rotation)
            );
        }

        // HEADING LOCK: Only lock heading when robot is stationary
        if (!isRobotMoving) {
            double[] lockPowers = RunHeadingLock();
            if (lockPowers != null) {
                 drivetrain.ApplyHeadingLockPowers(lockPowers);
            }
        }
    }

    /**
     * Update robot position from Pinpoint odometry computer
     * This runs every loop to continuously track robot position on the field
     */
    private void UpdatePositionFromPinpoint() {
        // Update Pinpoint odometry readings
        pinpoint.update();

        // Get current position in INCHES (Pedro Pathing standard units)
        Pose2D pose = pinpoint.getPosition();
        robotX = pose.getX(DistanceUnit.INCH);
        robotY = pose.getY(DistanceUnit.INCH);

        // Debug telemetry
        telemetry.Log("=== POSITION DEBUG ===", "");
        telemetry.Log("Robot Position", String.format("X:%.1f\" Y:%.1f\"", robotX, robotY));
        telemetry.Log("Target Position", String.format("X:%.1f\" Y:%.1f\"", targetX, targetY));
        telemetry.Log("Delta to Target", String.format("ΔX:%.1f\" ΔY:%.1f\"", targetX - robotX, targetY - robotY));
        telemetry.Log("Distance", String.format("%.2fm (%.1fin)", GetDistanceToTarget(), GetDistanceToTarget() / 0.0254));

        // Heading debug
        double targetHeading = GetHeadingToTarget();
        double currentHeading = GetCurrentHeading();
        double headingError = NormalizeAngle(targetHeading - currentHeading);
        telemetry.Log("=== HEADING DEBUG ===", "");
        telemetry.Log("Current Heading", String.format("%.1f°", currentHeading));
        telemetry.Log("Target Heading", String.format("%.1f°", targetHeading));
        telemetry.Log("Heading Error", String.format("%.1f°", headingError));
        telemetry.Log("Heading Lock", headingLockEnabled ? "ENABLED" : "DISABLED");
    }

    /**
     * Read and process button inputs
     */
    private void ReadButtons() {
        // Only process buttons if they exist (teleop mode)
        if (resetPositionButton == null) return;

        resetPositionButton.readValue();
        toggleHeadingLockButton.readValue();

        // DPAD_DOWN: Reset robot position to default (9, 9) in INCHES
        if (resetPositionButton.wasJustPressed()) {
            // Reset Pinpoint position AND recalibrate IMU for best accuracy
            // Use 90° heading to match autonomous starting pose (90° = facing forward in Pedro Pathing)
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 9, 9, AngleUnit.DEGREES, 90));
            pinpoint.recalibrateIMU();  // CRITICAL: Recalibrate IMU to reduce drift

            // Update local variables
            robotX = 9;
            robotY = 9;

            // Reset backup IMU too
            imu.resetYaw();

            telemetry.Log("Position Reset", "X:9\" Y:9\" Heading:90° (IMU recalibrated)");
        }

        // DPAD_LEFT: Toggle heading lock on/off
        if (toggleHeadingLockButton.wasJustPressed()) {
            if (headingLockEnabled) {
                UnlockHeading();
            } else {
                LockCurrentHeading();
            }
        }

    }

    /**
     * Calculate distance from robot to target
     * Robot and target positions are in INCHES (Pedro Pathing units)
     * Returns distance in METERS (for shooter calculations)
     * @return Distance in meters
     */
    public double GetDistanceToTarget()
    {
        // Calculate relative position from robot to target (both in INCHES)
        double deltaX = targetX - robotX;  // Positive if target is to the right
        double deltaY = targetY - robotY;  // Positive if target is forward

        // Calculate distance in inches using Pythagorean theorem
        double distanceInInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Convert to METERS (1 inch = 0.0254 m) for shooter calculations
        return distanceInInches * 0.0254;  // Return in METERS
    }

    /**
     * Calculate the heading angle (in degrees) from robot's current position to target
     * This creates a straight line from the front of the robot to the target point
     * Uses PEDRO PATHING coordinate system: 0° = right, 90° = forward, 180° = left, 270° = back
     * @return Heading angle in degrees (-180 to 180)
     */
    public double GetHeadingToTarget()
    {
        // Calculate relative position from robot to target
        double deltaX = targetX - robotX;  // X difference
        double deltaY = targetY - robotY;  // Y difference

        // Calculate angle using atan2 for Pedro Pathing coordinate system
        // Pedro Pathing: 0° = right (X+), 90° = forward (Y+), 180° = left (X-), 270° = back (Y-)
        // atan2(y, x) returns angle from positive X-axis, which matches Pedro's system
        double angleRadians = Math.atan2(deltaY, deltaX);

        // Convert to degrees
        double angleDegrees = Math.toDegrees(angleRadians);

        // Normalize to -180 to 180 range
        return NormalizeAngle(angleDegrees);
    }


    /**
     * Reset robot position to default starting position (9, 9) in inches
     * Call this when button is pressed to calibrate position
     * Preserves current heading - use CalibrateHeading() separately to reset heading to 0°
     */
    public void ResetRobotPosition()
    {
        this.robotX = 9.0;
        this.robotY = 9.0;
        // Reset Pinpoint position while preserving current heading (in INCHES)
        double currentHeading = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 9.0, 9.0, AngleUnit.DEGREES, currentHeading));
        telemetry.Log("Position Reset", String.format("X:9.0\" Y:9.0\" Heading:%.1f°", currentHeading));
    }


    // ==================== HEADING LOCK FEATURE ====================

    /**
     * Enable heading lock towards target point
     * Robot will continuously try to point towards the target (straight line from front to target)
     * This maintains orientation even if robot is hit by another robot
     */
    public void LockCurrentHeading() {
        lockedHeading = GetHeadingToTarget();  // Lock to heading towards target
        headingLockEnabled = true;
        telemetry.Log("Heading Lock", String.format("ENABLED - Pointing to target at %.1f°", lockedHeading));
    }

    /**
     * Disable heading lock
     * Robot will stop trying to maintain heading
     */
    public void UnlockHeading() {
        headingLockEnabled = false;
        telemetry.Log("Heading Lock", "DISABLED");
    }

    /**
     * Check if heading lock is active
     */
    public boolean IsHeadingLocked() {return headingLockEnabled;}


    /**
     * HEADING LOCK FEATURE - Run heading lock correction
     * Continuously points robot towards target point (straight line from front to target)
     * Updates target heading in real-time as robot or target moves
     * Manual controls (joystick/triggers) have priority and temporarily disable this
     * @return Motor powers if heading lock is active, null otherwise
     */
    private double[] RunHeadingLock()
    {
        if (!headingLockEnabled) return null;

        // Continuously update locked heading to point towards target
        // This ensures robot always aims at target, even if position changes
        lockedHeading = GetHeadingToTarget();

        // Get current robot heading from Pinpoint (for consistency with position)
        double currentAngle = GetCurrentHeading();
        double error = NormalizeAngle(lockedHeading - currentAngle);

        // Small deadband to avoid micro-corrections
        if (Math.abs(error) < 1.0) {
            return new double[]{0, 0, 0, 0}; // Within 1°, no correction needed
        }

        // Simple P control for heading lock (no I or D to keep it smooth)
        double rotationPower = kP * error * 0.3;  // 30% of normal P gain for gentler correction
        rotationPower = Math.max(-0.3, Math.min(0.3, rotationPower)); // Clamp to ±0.3

        // Return rotation powers to point towards target
        return new double[]{
            -rotationPower,  // leftFront
            rotationPower,   // rightFront
            -rotationPower,  // leftBack
            rotationPower    // rightBack
        };
    }


    /**
     * Set whether robot is currently being controlled manually
     * Manual controls (joystick & triggers) have PRIORITY over heading lock
     * Heading lock only applies when robot is NOT being manually controlled
     * @param leftStickX Left joystick X input (strafe)
     * @param leftStickY Left joystick Y input (forward/back)
     * @param leftTrigger Left trigger (rotate left)
     * @param rightTrigger Right trigger (rotate right)
     */
    public void SetRobotMoving(double leftStickX, double leftStickY, double leftTrigger, double rightTrigger) {
        // Robot is considered "moving" if ANY controller input exceeds threshold
        // This gives MANUAL CONTROLS absolute priority over heading lock
        boolean wasMoving = isRobotMoving;
        double threshold = 0.05;
        isRobotMoving = Math.abs(leftStickX) > threshold ||
                       Math.abs(leftStickY) > threshold ||
                       Math.abs(leftTrigger) > threshold ||
                       Math.abs(rightTrigger) > threshold;

        // When manual control starts, heading lock pauses (but stays enabled)
        if (isRobotMoving && !wasMoving && headingLockEnabled) {
            telemetry.Log("Heading Lock", "Paused - Manual control active");
        }

        // When manual control stops, heading lock resumes pointing at target
        if (!isRobotMoving && wasMoving && headingLockEnabled) {
            lockedHeading = GetHeadingToTarget();
            telemetry.Log("Heading Lock", String.format("Resumed - Pointing to target at %.1f°", lockedHeading));
        }
    }

    /**
     * Normalize angle to -180 to 180 range
     */
    private double NormalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Get current robot heading from Pinpoint odometry
     * Uses Pinpoint's IMU for consistency with position tracking
     */
    public double GetCurrentHeading() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }


    /**
     * Get robot's current X position
     */
    public double GetRobotX() {return robotX;}

    /**
     * Get robot's current Y position
     */
    public double GetRobotY() {
        return robotY;
    }
}

