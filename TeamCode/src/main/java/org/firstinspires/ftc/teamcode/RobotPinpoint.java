package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.PinpointBlocksDriver.GoBildaPinpointDriver;

public class RobotPinpoint implements Subsystem {
    private GoBildaPinpointDriver pinpoint;
    private TelemetryCustom telemetry;

    // Button reader for reset
    private ButtonReader resetButton;

    // Current position
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    // Initial position (for reset)
    private double initialX = 0;
    private double initialY = 0;
    private double initialHeading = 0;

    public RobotPinpoint(TelemetryCustom telemetry, GamepadEx gamepad) {
        this.telemetry = telemetry;

        // Button for reset (example: DPAD_DOWN on gamepad)
        if (gamepad != null) {
            resetButton = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_DOWN);
        }
    }

    public void LinkComponents(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public void Initialize(HardwareMap hardwareMap) {
        LinkComponents(hardwareMap);
        ConfigurePinpoint();

        // NU resetăm poziția - păstrăm poziția din Auto!
        // pinpoint.resetPosAndIMU();
        // pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, initialX, initialY, AngleUnit.DEGREES, initialHeading));
        // pinpoint.recalibrateIMU();

        telemetry.Log("RobotPinpoint", "Initialized - Position preserved from Auto");
    }

    private void ConfigurePinpoint() {
        // Configure pinpoint offsets (adjust these to your robot)
        // X offset and Y offset in inches
        pinpoint.setOffsets(2.11, -3.31);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
    }

    public void Run() {
        // Update pinpoint position
        UpdatePosition();

        // Check for reset button press
        if (resetButton != null) {
            resetButton.readValue();

            if (resetButton.wasJustPressed()) {
                ResetPosition();
            }
        }
    }

    /**
     * Update current position from pinpoint
     */
    private void UpdatePosition() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        x = pose.getX(DistanceUnit.INCH);
        y = pose.getY(DistanceUnit.INCH);
        heading = pose.getHeading(AngleUnit.DEGREES);
    }

    /**
     * Reset pinpoint position to initial values
     */
    public void ResetPosition() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, initialX, initialY, AngleUnit.DEGREES, initialHeading));
        pinpoint.recalibrateIMU();

        x = initialX;
        y = initialY;
        heading = initialHeading;

        telemetry.Log("Pinpoint Reset", String.format("X:%.1f\" Y:%.1f\" H:%.1f°", initialX, initialY, initialHeading));
    }

    /**
     * Set the initial position (used for reset)
     */
    public void SetInitialPosition(double x, double y, double headingDegrees) {
        this.initialX = x;
        this.initialY = y;
        this.initialHeading = headingDegrees;
    }

    /**
     * Set current position (for relocalization)
     */
    public void SetPosition(double x, double y, double headingDegrees) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDegrees));
        this.x = x;
        this.y = y;
        this.heading = headingDegrees;
    }

    // Getters
    public double GetX() { return x; }
    public double GetY() { return y; }
    public double GetHeading() { return heading; }
    public double GetHeadingRadians() { return Math.toRadians(heading); }

    public GoBildaPinpointDriver GetPinpoint() { return pinpoint; }

    public void UpdateTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("=== PINPOINT ===");
        telemetry.addData("  X", "%.2f\"", x);
        telemetry.addData("  Y", "%.2f\"", y);
        telemetry.addData("  Heading", "%.1f°", heading);
    }
}
