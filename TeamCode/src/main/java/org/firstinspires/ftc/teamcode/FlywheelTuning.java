package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Flywheel/Shooter Tuning OpMode (without FTC Dashboard)
 *
 * Instructions:
 * 1. Enable this OpMode (remove @Disabled)
 * 2. Use gamepad controls to adjust parameters:
 *    - Dpad Up/Down: Increase/Decrease target velocity
 *    - Y/A: Increase/Decrease kV
 *    - B/X: Increase/Decrease kS
 *    - Right Bumper/Trigger: Increase/Decrease P
 * 3. Watch telemetry for current velocity vs target
 * 4. Record final kV, kS, P values for use in Shooter.java
 */
@TeleOp(name = "Flywheel Tuning (kV/kS)", group = "Tuning")
@Disabled  // Remove this when ready to tune
public class FlywheelTuning extends OpMode {
    private PIDFController controller;
    private DcMotorEx motor1, motor2;

    // Tunable parameters
    private double targetVelocity = 1700;  // Target RPM
    private double kV = 0.0;               // Velocity feedforward
    private double kS = 0.0;               // Static friction
    private double P = 0.0;                // Proportional gain
    private double I = 0.0;                // Integral gain (usually 0.0)

    // Adjustment increments
    private static final double VELOCITY_INCREMENT = 50;
    private static final double KV_INCREMENT = 0.0001;
    private static final double KS_INCREMENT = 0.01;
    private static final double P_INCREMENT = 0.001;

    private double velocity = 0.0;
    private double error = 0.0;

    // Button debouncing
    private boolean lastDpadUp = false, lastDpadDown = false;
    private boolean lastY = false, lastA = false;
    private boolean lastB = false, lastX = false;
    private boolean lastRightBumper = false, lastRightTrigger = false;

    @Override
    public void init() {
        // Initialize both shooter motors (same as Shooter.java)
        motor1 = hardwareMap.get(DcMotorEx.class, "MotorAruncare1");
        motor2 = hardwareMap.get(DcMotorEx.class, "MotorAruncare2");

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize PIDF controller
        controller = new PIDFController(P, I, 0.0, 0.0);

        telemetry.addLine("✓ Flywheel Tuning Ready");
        telemetry.addLine("Use gamepad to adjust parameters");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Handle gamepad input for parameter tuning
        handleGamepadInput();

        // Get current velocity from motor 1
        velocity = motor1.getVelocity();
        error = targetVelocity - velocity;

        // Update PIDF coefficients (F = kV * target + kS)
        double feedforward = kV * targetVelocity + kS;
        controller.setPIDF(P, I, 0.0, feedforward);

        // Calculate motor power
        double power = controller.calculate(error);

        // Apply power to both motors
        motor1.setPower(power);
        motor2.setPower(power);

        // Telemetry for tuning
        telemetry.addData("═══ VELOCITY ═══", "");
        telemetry.addData("Target", "%.0f RPM", targetVelocity);
        telemetry.addData("Current", "%.0f RPM", velocity);
        telemetry.addData("Error", "%.0f RPM", error);
        telemetry.addLine();

        telemetry.addData("═══ TUNING PARAMS ═══", "");
        telemetry.addData("kV", "%.5f (Y↑/A↓)", kV);
        telemetry.addData("kS", "%.3f (B↑/X↓)", kS);
        telemetry.addData("P", "%.4f (RB↑/RT↓)", P);
        telemetry.addLine();

        telemetry.addData("═══ OUTPUT ═══", "");
        telemetry.addData("Feedforward", "%.3f", feedforward);
        telemetry.addData("Motor Power", "%.3f", power);
        telemetry.addLine();

        telemetry.addData("═══ CONTROLS ═══", "");
        telemetry.addData("Target Vel", "DPad Up/Down");
        telemetry.update();
    }

    private void handleGamepadInput() {
        // Target velocity adjustment
        if (gamepad1.dpad_up && !lastDpadUp) {
            targetVelocity += VELOCITY_INCREMENT;
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            targetVelocity = Math.max(0, targetVelocity - VELOCITY_INCREMENT);
        }

        // kV adjustment
        if (gamepad1.y && !lastY) {
            kV += KV_INCREMENT;
        }
        if (gamepad1.a && !lastA) {
            kV = Math.max(0, kV - KV_INCREMENT);
        }

        // kS adjustment
        if (gamepad1.b && !lastB) {
            kS += KS_INCREMENT;
        }
        if (gamepad1.x && !lastX) {
            kS = Math.max(0, kS - KS_INCREMENT);
        }

        // P adjustment
        if (gamepad1.right_bumper && !lastRightBumper) {
            P += P_INCREMENT;
        }
        if (gamepad1.right_trigger > 0.5 && !lastRightTrigger) {
            P = Math.max(0, P - P_INCREMENT);
        }

        // Update button states for debouncing
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastY = gamepad1.y;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastRightBumper = gamepad1.right_bumper;
        lastRightTrigger = gamepad1.right_trigger > 0.5;
    }

    @Override
    public void stop() {
        // Stop motors and print final tuned values
        motor1.setPower(0);
        motor2.setPower(0);

        telemetry.addLine("═══════════════════════════");
        telemetry.addLine("FINAL TUNED VALUES:");
        telemetry.addData("kV", kV);
        telemetry.addData("kS", kS);
        telemetry.addData("P", P);
        telemetry.addLine("═══════════════════════════");
        telemetry.update();
    }
}
