package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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
    private ButtonReader relocalizationButton;  // NEW: Button to trigger relocalization
    private ButtonReader toggleAbsoluteHeadingLockButton;  // NEW: Button for absolute heading lock

    // PID coefficients
    private final double kP;
    private final double kP_position = 0.09;

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
        this.kP = 0.65;
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
        this.kP = 0.65;
    }

    public void LinkComponents(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Only initialize button readers if gamepads exist (teleop mode)
        if (ct1 != null && ct2 != null) {
            resetPositionButton = new ButtonReader(ct2, GamepadKeys.Button.DPAD_DOWN);
            toggleHeadingLockButton = new ButtonReader(ct1, GamepadKeys.Button.RIGHT_BUMPER);
            relocalizationButton = new ButtonReader(ct2, GamepadKeys.Button.LEFT_BUMPER);  // ct2 left bumper
            toggleAbsoluteHeadingLockButton = new ButtonReader(ct2, GamepadKeys.Button.RIGHT_BUMPER);  // ct2 right bumper
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
        pinpoint.setOffsets(-3.62, -6.65, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        telemetry.Log("Pinpoint", "Configured: X=-3.62\" Y=-6.65\" (4-bar pods)");
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

        // ABSOLUTE HEADING LOCK (prevents rotation, allows translation)
        if (absoluteHeadingLockEnabled && drivetrain != null) {
            double[] lockPowers = RunAbsoluteHeadingLock();
            if (lockPowers != null) {
                drivetrain.ApplyHeadingLockPowers(lockPowers);
            }
        }
        // TARGET HEADING LOCK (only when not moving and not using absolute lock)
        else if (headingLockEnabled && !isRobotMoving && drivetrain != null) {
            double[] lockPowers = RunHeadingLock();
            if (lockPowers != null) {
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
                // Near top edge - shift target down by 3 inches
                targetX = 0;
                targetY = 143.6 - 3.0;
            }
            else if (robotY < 23.0)
            {
                // Near bottom edge - shift target right by 3 inches
                targetX = 0 + 3.0;
                targetY = 143.6;
            }
            else
            {
                // Normal position (between 23 and 110) - use default target
                targetX = 0;
                targetY = 143.6;
            }
        }
        else
        {
            // RED TEAM TARGET ADJUSTMENT
            if (robotY > 110.0)
            {
                // Near top edge - shift target down by 3 inches
                targetX = 143.6;
                targetY = 143.6 - 3.0;
            }
            else if (robotY < 23.0)
            {
                // Near bottom edge - shift target left by 3 inches
                targetX = 143.6 - 3.0;
                targetY = 143.6;
            }
            else
            {
                // Normal position (between 23 and 110) - use default target
                targetX = 143.6;
                targetY = 143.6;
            }
        }
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
            ct2.gamepad.rumble(500);  // Quick rumble to indicate failure
            return;
        }

        // ADDED: Sanity check on calculated position
        double deltaX = Math.abs(pose.x - robotX);
        double deltaY = Math.abs(pose.y - robotY);
        double deltaH = Math.abs(pose.heading - GetCurrentHeading());

        // If position jumps more than 24 inches or heading jumps more than 30°, reject it
        if (deltaX > 24.0 || deltaY > 24.0 || deltaH > 30.0) {
            telemetry.Log("⚠️ REJECTED Reloc", String.format("ΔX=%.1f ΔY=%.1f ΔH=%.1f", deltaX, deltaY, deltaH));
            ct2.gamepad.rumble(1000);  // Longer rumble to indicate rejection
            return;  // Don't apply this relocalization - it's too big a jump
        }

        // Position change is reasonable - apply it
        Pose2D relocPose = new Pose2D(DistanceUnit.INCH, pose.x, pose.y, AngleUnit.DEGREES, pose.heading);
        pinpoint.setPosition(relocPose);
        robotX = pose.x;
        robotY = pose.y;

        // Success feedback
        ct2.gamepad.rumble(0.6, 0.6, 300);  // Short double-rumble for success
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
        toggleAbsoluteHeadingLockButton.readValue();  // NEW: Read absolute heading lock button

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

        // NEW: Absolute heading lock button pressed
        if (toggleAbsoluteHeadingLockButton.wasJustPressed()) {
            if (absoluteHeadingLockEnabled) {
                UnlockAbsoluteHeading();
            } else {
                LockAbsoluteHeading();
            }
        }

        // Relocalization button pressed
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
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double distanceInInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        return distanceInInches * 0.0254;
    }

    public double GetHeadingToTarget()
    {
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

    public void SyncPositionFromPedro(double x, double y, double headingDegrees)
    {
        this.robotX = x;
        this.robotY = y;
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDegrees));
    }

    // Heading Lock
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
    // Constantly points robot at target, prevents manual rotation

    /**
     * Lock heading towards target - robot constantly faces target even while moving
     * Prevents manual rotation
     */
    public void LockAbsoluteHeading()
    {
        absoluteHeadingLockEnabled = true;
        telemetry.Log("🔒 Auto-Aim Lock", "ENABLED - Always facing target");
    }

    /**
     * Unlock absolute heading - robot can rotate freely
     */
    public void UnlockAbsoluteHeading()
    {
        absoluteHeadingLockEnabled = false;
        telemetry.Log("🔓 Auto-Aim Lock", "DISABLED");
    }

    public boolean IsAbsoluteHeadingLocked() {return absoluteHeadingLockEnabled;}

    /**
     * Calculate correction powers to constantly point at target
     * Works even while robot is moving - prevents manual rotation
     */
    private double[] RunAbsoluteHeadingLock()
    {
        if (!absoluteHeadingLockEnabled) return null;

        // Calculate heading to target (same as target heading lock)
        double targetHeading = GetHeadingToTarget();
        double currentAngle = GetCurrentHeading();
        double error = NormalizeAngle(targetHeading - currentAngle);

        // Allow small heading errors without correction
        if (Math.abs(error) < 1.0) {
            return null;
        }

        // PID-style correction
        double errorSign = Math.signum(error);
        double errorMagnitude = Math.abs(error);
        double scaledError = errorSign * Math.pow(errorMagnitude / 180.0, 1.5) * 180.0;
        double rotationPower = kP * scaledError * 0.15;
        rotationPower = Math.max(-0.35, Math.min(0.35, rotationPower));

        return new double[]{
                rotationPower,
                -rotationPower,
                rotationPower,
                -rotationPower
        };
    }

    // ==================== TARGET HEADING LOCK (ORIGINAL) ====================

    private double[] RunHeadingLock()
    {
        if (!headingLockEnabled) return null;

        lockedHeading = GetHeadingToTarget();
        double currentAngle = GetCurrentHeading();
        double error = NormalizeAngle(lockedHeading - currentAngle);

        if (Math.abs(error) < 3.0) {
            return null;
        }

        double errorSign = Math.signum(error);
        double errorMagnitude = Math.abs(error);
        double scaledError = errorSign * Math.pow(errorMagnitude / 180.0, 1.5) * 180.0;
        double rotationPower = kP * scaledError * 0.15;
        rotationPower = Math.max(-0.2, Math.min(0.2, rotationPower));

        return new double[]{
                rotationPower,
                -rotationPower,
                rotationPower,
                -rotationPower
        };
    }

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
}