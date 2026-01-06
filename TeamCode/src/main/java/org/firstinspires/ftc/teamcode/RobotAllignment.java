package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RobotAllignment implements Subsystem
{
    private GoBildaPinpointDriver pinpoint;  // Odometry computer for position tracking (includes IMU)
    private final TelemetryCustom telemetry;
    private GamepadEx ct1, ct2;
    private Drivetrain drivetrain;  // Reference to drivetrain for applying heading lock

    // Button readers
    private ButtonReader resetPositionButton;
    private ButtonReader toggleHeadingLockButton;

    // PID coefficients for rotation - using Pedro Pathing's tuned heading PID
    private final double kP;
    // PID coefficients for position hold
    private final double kP_position = 0.09;  // Proportional gain for position (gentler than rotation)

    // Target position for distance calculation
    // (in INCHES - Pedro Pathing units) X:
    private double targetX = 0;
    private double targetY = 143;

    // Robot's current position on field (in INCHES - Pedro Pathing units)
    private double robotX = 9.000;  // Default starting X position in inches
    private double robotY = 9.000;    // Default starting Y position in inches

    // Heading lock feature - maintains heading continuously (only when stationary)
    private boolean headingLockEnabled = false;  // DISABLED by default - press DPAD_LEFT to enable
    private double lockedHeading = 0;
    private boolean isRobotMoving = false;  // Track if robot is currently moving

    // Strafe lock feature - maintains heading while moving (strafing/forward/back allowed)
    private boolean strafeLockEnabled = false;  // DISABLED by default - press DPAD_RIGHT to enable
    private double strafeLockedHeading = 0;

    // Position lock feature - maintains X,Y position when hit by another robot
    private boolean positionLockEnabled = false;  // DISABLED by default - press DPAD_UP to enable
    private double lockedX = 0;
    private double lockedY = 0;

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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Only initialize button readers if gamepads exist (teleop mode)
        if (ct1 != null && ct2 != null) {
            resetPositionButton = new ButtonReader(ct2, GamepadKeys.Button.DPAD_DOWN);
            toggleHeadingLockButton = new ButtonReader(ct1, GamepadKeys.Button.RIGHT_BUMPER);
        }
    }

    public void Initialize(HardwareMap hardwareMap) {
        LinkComponents(hardwareMap);

        // ===== CRITICAL: Configure Pinpoint Odometry Computer =====
        ConfigurePinpoint();

        // Optionally reset position and IMU (useful for autonomous start)
        if (resetIMUOnInit)
        {
            pinpoint.resetPosAndIMU();  // Resets position to (0,0,0) and recalibrates IMU
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 9, 9, AngleUnit.DEGREES, 90));
            pinpoint.recalibrateIMU();
            robotX = 9;
            robotY = 9;
            telemetry.Log("RobotAllignment", "Pinpoint reset: X:9\" Y:9\" Heading:90°");
        }
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

        // PRIORITY ORDER: Position Lock > Heading Lock > Strafe Lock
        // Position lock takes highest priority (actively drives back to position)
        // POSITION LOCK: Maintain X,Y position when hit (commented out by default)
        /*
        if (positionLockEnabled && !isRobotMoving && drivetrain != null) {
            // Robot is stationary AND position locked - maintain position
            double[] positionPowers = RunPositionLock();
            if (positionPowers != null) {
                drivetrain.ApplyMovementPowers(positionPowers);
                return; // Skip other locks
            }
        }
        */

        // HEADING LOCK: Continuously point towards target when stationary
        if (headingLockEnabled && !isRobotMoving && drivetrain != null) {
            // Robot is stationary - apply heading lock to point at target
            double[] lockPowers = RunHeadingLock();
            if (lockPowers != null) {
                drivetrain.ApplyHeadingLockPowers(lockPowers);
            }
        }
        // STRAFE LOCK: Maintain heading while moving (allows movement but keeps pointing at target)
        // Comment out the lines below to disable strafe lock
        /*
        else if (strafeLockEnabled && isRobotMoving && drivetrain != null) {
            // Robot is moving - apply strafe lock to maintain heading towards target
            double[] lockPowers = RunStrafeLock();
            if (lockPowers != null) {
                drivetrain.ApplyHeadingLockPowers(lockPowers);
            }
        }
        */
        else if (drivetrain != null) {
            // All locks disabled - clear corrections
            drivetrain.ApplyHeadingLockPowers(null);
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
        telemetry.Log("Strafe Lock", strafeLockEnabled ? "ENABLED" : "DISABLED");
        telemetry.Log("Position Lock", positionLockEnabled ? String.format("ENABLED (X:%.1f\" Y:%.1f\")", lockedX, lockedY) : "DISABLED");
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


            telemetry.Log("Position Reset", "X:9\" Y:9\" Heading:90° (IMU recalibrated)");
        }

        // DPAD_LEFT: Toggle heading lock on/off (only works when stationary)
        if (toggleHeadingLockButton.wasJustPressed()) {
            if (headingLockEnabled) {
                UnlockHeading();
            } else {
                LockCurrentHeading();
            }
        }
/*
        // DPAD_RIGHT: Toggle strafe lock on/off (maintains heading while moving)
        if (toggleStrafeLockButton.wasJustPressed()) {
            if (strafeLockEnabled) {
                UnlockStrafeLock();
            } else {
                LockStrafeLock();
            }
        }

        // DPAD_UP: Toggle position lock on/off (maintains X,Y position when hit)
        if (togglePositionLockButton.wasJustPressed()) {
            if (positionLockEnabled) {
                UnlockPosition();
            } else {
                LockCurrentPosition();
            }
        }
*/
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

    public void LockCurrentHeading()
    {
        lockedHeading = GetHeadingToTarget();  // Lock to heading towards target
        headingLockEnabled = true;
        telemetry.Log("Heading Lock", String.format("ENABLED - Pointing to target at %.1f°", lockedHeading));
    }
    public void UnlockHeading()
    {
        headingLockEnabled = false;
        telemetry.Log("Heading Lock", "DISABLED");
    }

    public boolean IsHeadingLocked() {return headingLockEnabled;}


    // ==================== STRAFE LOCK FEATURE ====================

    public void LockStrafeLock()
    {
        strafeLockedHeading = GetHeadingToTarget();  // Lock to heading towards target
        strafeLockEnabled = true;
        telemetry.Log("Strafe Lock", String.format("ENABLED - Maintaining heading to target at %.1f°", strafeLockedHeading));
    }

    public void UnlockStrafeLock()
    {
        strafeLockEnabled = false;
        telemetry.Log("Strafe Lock", "DISABLED");
    }
    public boolean IsStrafeLocked() {return strafeLockEnabled;}

    private double[] RunStrafeLock()
    {
        if (!strafeLockEnabled) return null;

        // Continuously update locked heading to point towards target
        // This ensures robot always aims at target, even as it moves around the field
        strafeLockedHeading = GetHeadingToTarget();

        // Get current robot heading from Pinpoint
        double currentAngle = GetCurrentHeading();
        double error = NormalizeAngle(strafeLockedHeading - currentAngle);

        // LARGER deadband to prevent oscillation while moving
        if (Math.abs(error) < 3.0) {
            return null; // Within 5°, no correction needed - return null to clear corrections
        }

        // Exponential scaling for smoother correction while moving
        double errorSign = Math.signum(error);
        double errorMagnitude = Math.abs(error);
        double scaledError = errorSign * Math.pow(errorMagnitude / 180.0, 1.5) * 180.0;

        // Even gentler P gain for strafe lock (12.5% instead of 25%)
        double rotationPower = kP * scaledError * 0.125;

        // Clamp to ±0.15 for very gentle correction while moving
        rotationPower = Math.max(-0.15, Math.min(0.15, rotationPower));

        // Return rotation powers to maintain heading towards target
        // Pattern matches Drivetrain.Rotate(): FS=+power, FD=-power, SS=+power, SD=-power
        return new double[]{
            rotationPower,   // leftFront (FS)
            -rotationPower,  // rightFront (FD)
            rotationPower,   // leftBack (SS)
            -rotationPower   // rightBack (SD)
        };
    }


    // ==================== POSITION LOCK FEATURE ====================

    /**
     * Enable position lock at current location
     * Robot will actively drive back to this position if pushed by another robot
     * Only works when stationary (manual controls override)
     */
    public void LockCurrentPosition() {
        lockedX = robotX;  // Lock current X position
        lockedY = robotY;  // Lock current Y position
        positionLockEnabled = true;
        telemetry.Log("Position Lock", String.format("ENABLED at X:%.1f\" Y:%.1f\"", lockedX, lockedY));
    }

    /**
     * Disable position lock
     * Robot will stop trying to maintain position
     */
    public void UnlockPosition() {
        positionLockEnabled = false;
        telemetry.Log("Position Lock", "DISABLED");
    }

    /**
     * Check if position lock is active
     */
    public boolean IsPositionLocked() {return positionLockEnabled;}

    /**
     * POSITION LOCK FEATURE - Maintain X,Y position when robot is pushed
     * Actively drives robot back to locked position using mecanum drive
     * Uses field-centric control to correct position drift
     * @return Motor powers [FS, FD, SS, SD] for all 4 motors, null if lock disabled
     */
    private double[] RunPositionLock()
    {
        if (!positionLockEnabled) return null;

        // Calculate position error (how far we've been pushed from locked position)
        double errorX = lockedX - robotX;  // Error in X (inches)
        double errorY = lockedY - robotY;  // Error in Y (inches)

        // Calculate total position error magnitude
        double totalError = Math.sqrt(errorX * errorX + errorY * errorY);

        // Deadband: if within 1 inch, no correction needed
        if (totalError < 1.0) {
            return new double[]{0, 0, 0, 0};
        }

        // Convert field-relative error to robot-relative using current heading
        double currentHeading = GetCurrentHeading();
        double headingRad = Math.toRadians(currentHeading);

        // Rotate error vector by -heading to get robot-relative coordinates
        // This makes the correction field-centric (always drives toward locked position)
        double robotRelativeX = errorX * Math.cos(-headingRad) - errorY * Math.sin(-headingRad);
        double robotRelativeY = errorX * Math.sin(-headingRad) + errorY * Math.cos(-headingRad);

        // P control for position correction (proportional to error)
        double powerX = kP_position * robotRelativeX;  // Strafe power
        double powerY = kP_position * robotRelativeY;  // Forward/back power

        // Clamp individual powers to ±0.3 for gentle correction
        powerX = Math.max(-0.3, Math.min(0.3, powerX));
        powerY = Math.max(-0.3, Math.min(0.3, powerY));

        // Convert strafe/forward powers to individual motor powers (mecanum drive)
        // Mecanum formula: FS = Y + X, FD = Y - X, SS = Y - X, SD = Y + X
        double frontLeft = powerY + powerX;   // FS (MotorFS)
        double frontRight = powerY - powerX;  // FD (MotorFD)
        double backLeft = powerY - powerX;    // SS (MotorSS)
        double backRight = powerY + powerX;   // SD (MotorSD)

        // Find max power to normalize if needed (prevent any motor from exceeding 1.0)
        double maxPower = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                                  Math.max(Math.abs(backLeft), Math.abs(backRight)));
        if (maxPower > 0.3) {
            double scale = 0.3 / maxPower;
            frontLeft *= scale;
            frontRight *= scale;
            backLeft *= scale;
            backRight *= scale;
        }

        // Return motor powers for position correction
        return new double[]{frontLeft, frontRight, backLeft, backRight};
    }


    private double[] RunHeadingLock()
    {
        if (!headingLockEnabled) return null;

        // Continuously update locked heading to point towards target
        // This ensures robot always aims at target, even if position changes
        lockedHeading = GetHeadingToTarget();

        // Get current robot heading from Pinpoint (for consistency with position)
        double currentAngle = GetCurrentHeading();
        double error = NormalizeAngle(lockedHeading - currentAngle);

        // LARGER deadband to prevent oscillation (5° is more forgiving)
        if (Math.abs(error) < 3.0) {
            return null; // Within 5°, no correction needed - return null to clear corrections
        }

        // Exponential scaling: small errors get even smaller corrections
        // This prevents aggressive micro-corrections near the target
        double errorSign = Math.signum(error);
        double errorMagnitude = Math.abs(error);

        // Quadratic scaling for smoother approach (error^1.5 instead of linear)
        double scaledError = errorSign * Math.pow(errorMagnitude / 180.0, 1.5) * 180.0;

        // Reduced P gain (15% instead of 30%) for very gentle correction
        double rotationPower = kP * scaledError * 0.15;

        // Clamp to ±0.2 (reduced from 0.3) for slower, smoother rotation
        rotationPower = Math.max(-0.2, Math.min(0.2, rotationPower));

        // Return rotation powers to point towards target
        // Pattern matches Drivetrain.Rotate(): FS=+power, FD=-power, SS=+power, SD=-power
        return new double[]{
            rotationPower,   // leftFront (FS)
            -rotationPower,  // rightFront (FD)
            rotationPower,   // leftBack (SS)
            -rotationPower   // rightBack (SD)
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

