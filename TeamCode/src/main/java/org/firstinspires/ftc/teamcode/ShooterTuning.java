package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Shooter Tuning - Steps", group = "Tuning")
public class ShooterTuning extends OpMode {

    private DcMotorEx shooterMotor1, shooterMotor2;
    private LimeLight limelight;
    private TurretMechanismTutorial turret;
    private GamepadEx ct1;
    private ButtonReader nextStep, prevStep, increaseValue, decreaseValue;

    // Current tuning step
    private int currentStep = 0;
    private static final int TOTAL_STEPS = 2;

    // Step names
    private static final String[] STEP_NAMES = {
            "PIDF - F Value (Feedforward)",
            "Velocity Testing"
    };

    // PIDF values
    private double F = 15.16;
    private double P = 0.002; // Fixed
    private double I = 0.0; // Not used
    private double D = 0.0; // Not used

    // Test velocity
    private double testVelocity = 1000;

    @Override
    public void init() {
        ct1 = new GamepadEx(gamepad1);

        // Initialize Limelight
        limelight = new LimeLight(true, false);
        limelight.Initialize(hardwareMap);
        limelight.getLimelight().pipelineSwitch(2); // Blue pipeline

        // Initialize Turret
        turret = new TurretMechanismTutorial(limelight, null);
        turret.Initialize(hardwareMap);

        // Initialize shooter motors
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "MotorAruncare1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "MotorAruncare2");

        shooterMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Apply PIDF
        updatePIDF();

        // Button setup
        nextStep = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        prevStep = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        increaseValue = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        decreaseValue = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);

        telemetry.addLine("=============================");
        telemetry.addLine("  SHOOTER TUNING - READY");
        telemetry.addLine("=============================");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("• DPAD LEFT/RIGHT: Change step");
        telemetry.addLine("• DPAD UP/DOWN: Adjust value");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Read buttons
        nextStep.readValue();
        prevStep.readValue();
        increaseValue.readValue();
        decreaseValue.readValue();

        // Step navigation
        if (nextStep.wasJustPressed()) {
            currentStep = (currentStep + 1) % TOTAL_STEPS;
        }
        if (prevStep.wasJustPressed()) {
            currentStep = (currentStep - 1 + TOTAL_STEPS) % TOTAL_STEPS;
        }

        // Value adjustment based on current step
        if (increaseValue.wasJustPressed()) {
            adjustValue(true);
        }
        if (decreaseValue.wasJustPressed()) {
            adjustValue(false);
        }

        // Run turret and limelight
        limelight.Run();
        turret.Run();

        // Set motor velocity
        shooterMotor1.setVelocity(testVelocity);
        shooterMotor2.setVelocity(testVelocity);

        // Read actual velocities
        double vel1 = shooterMotor1.getVelocity();
        double vel2 = shooterMotor2.getVelocity();
        double avgVel = (vel1 + vel2) / 2.0;
        double error = testVelocity - avgVel;

        // Calculate distance
        double turretAngle = turret.getCurrentAngle();
        double rawDist = limelight.GetDistance2DToAprilTagFromRobotCenter(turretAngle);
        double POI_OFFSET_INCHES = 18.11;
        double finalDistInches = (rawDist > 0) ? (rawDist + POI_OFFSET_INCHES) : -1;
        double finalDistCm = (finalDistInches > 0) ? finalDistInches * 2.54 : -1;

        // Display telemetry
        displayTelemetry(avgVel, error, finalDistCm);
    }

    private void adjustValue(boolean increase) {
        double delta = increase ? 1 : -1;

        switch (currentStep) {
            case 0: // F value
                F += delta * 0.1;
                if (F < 0)
                    F = 0;
                updatePIDF();
                break;

            case 1: // Velocity
                testVelocity += delta * (increase ? 20 : -10);
                if (testVelocity < 0)
                    testVelocity = 0;
                break;
        }
    }

    private void updatePIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        shooterMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        shooterMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }

    private void displayTelemetry(double avgVel, double error, double distCm) {
        telemetry.addLine("============================");
        telemetry.addData(">>> STEP", String.format("%d/%d: %s",
                currentStep + 1, TOTAL_STEPS, STEP_NAMES[currentStep]));
        telemetry.addLine("============================");
        telemetry.addLine();

        // Current step highlight
        telemetry.addLine("--- CURRENT TUNING ---");
        switch (currentStep) {
            case 0:
                telemetry.addData(">> F", String.format("%.2f", F));
                telemetry.addData("   P (fixed)", String.format("%.4f", P));
                break;
            case 1:
                telemetry.addData("   F", String.format("%.2f", F));
                telemetry.addData(">> Target Vel", String.format("%.0f ticks/s", testVelocity));
                break;
        }
        telemetry.addLine();

        // Performance metrics
        telemetry.addLine("--- PERFORMANCE ---");
        telemetry.addData("Target Velocity", String.format("%.0f ticks/s", testVelocity));
        telemetry.addData("Actual Velocity", String.format("%.0f ticks/s", avgVel));
        telemetry.addData("Error", String.format("%.0f ticks/s (%.1f%%)",
                error, (testVelocity > 0 ? Math.abs(error / testVelocity) * 100 : 0)));
        telemetry.addLine();

        // Distance
        telemetry.addLine("--- DISTANCE ---");
        if (distCm > 0) {
            telemetry.addData("Distance", String.format("%.1f cm (%.1f in)",
                    distCm, distCm / 2.54));
        } else {
            telemetry.addData("Distance", "No Target");
        }
        telemetry.addLine();

        // Controls
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("DPAD ←/→: Change step");
        telemetry.addLine("DPAD ↑/↓: Adjust value");
        telemetry.addLine("B/X: Quick velocity adjust");

        telemetry.update();
    }
}