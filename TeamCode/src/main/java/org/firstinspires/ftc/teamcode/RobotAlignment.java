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

        // CHECK FOR TAG RELOCALIZATION (only in TeleOp when husky is available)
        if (enableRelocalization && husky != null) {
            CheckAndApplyRelocalization();
        }

        // Detect robot movement
        if (ct1 != null) {
            SetRobotMoving(
                    ct1.getLeftX(),
                    ct1.getLeftY(),
                    ct1.gamepad.left_trigger,
                    ct1.gamepad.right_trigger
            );
        }

        // Heading lock logic
        if (headingLockEnabled && !isRobotMoving && drivetrain != null) {
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
     * Check if Husky detected the team's AprilTag and apply relocalization
     * SIMPLIFIED - Husky does all the heavy lifting now!
     */
    // In RobotAlignment.java - pass current heading to Husky
    private void CheckAndApplyRelocalization() {
        if (husky == null) {
            return;
        }

        // Get CURRENT heading from IMU as initial guess
        double currentHeading = GetCurrentHeading();

        // Ask Husky to calculate pose using current heading as hint
        RobotPoseData pose = husky.CalculateRobotPose(currentHeading);

        // ADDED: Sanity check on calculated position
        if (pose != null) {
            // Check if position change is reasonable (not a huge jump)
            double deltaX = Math.abs(pose.x - robotX);
            double deltaY = Math.abs(pose.y - robotY);
            double deltaH = Math.abs(pose.heading - GetCurrentHeading());

            // If position jumps more than 24 inches or heading jumps more than 30°, reject it
            if (deltaX > 24.0 || deltaY > 24.0 || deltaH > 30.0) {
                telemetry.Log("⚠️ REJECTED Reloc", String.format("ΔX=%.1f ΔY=%.1f ΔH=%.1f", deltaX, deltaY, deltaH));
                return;  // Don't apply this relocalization - it's too big a jump
            }

            // Position change is reasonable - apply it
            Pose2D relocPose = new Pose2D(DistanceUnit.INCH, pose.x, pose.y, AngleUnit.DEGREES, pose.heading);
            pinpoint.setPosition(relocPose);
            robotX = pose.x;
            robotY = pose.y;

            telemetry.Log("✅ RELOCALIZED!", "");
            telemetry.Log("  New X", String.format("%.1f\"", robotX));
            telemetry.Log("  New Y", String.format("%.1f\"", robotY));
            telemetry.Log("  New Heading", String.format("%.1f°", pose.heading));
        }
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

    //Original poz/heading looking at the center of the tag before recalculating
    //X:74.78
    //Y:87.55
    //H:147.85

    //AfterRecalculating:

    //-Center
    //X:75.93
    //Y:85.38
    //H:146.9

    //Tagx 159
    //TagWidth 28
    //Horiz Distance: 71.2

    //-Left Part

    //X:76.67
    //Y:107
    //H:134.2

    //TagX 28
    //TagWidth: 40
    //HorizDistance: 61.4

    //-RightPart

    //X:70.9
    //Y:89.27
    //H:165.7

    //TagX 291
    //TagWidth: 38
    //HorizDistance: 64.7


    private double NormalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public double GetCurrentHeading() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    public double GetRobotX() {return robotX;}
    public double GetRobotY() {return robotY;}
}