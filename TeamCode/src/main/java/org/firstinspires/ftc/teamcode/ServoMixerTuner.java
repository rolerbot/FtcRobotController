package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hood Servo Tuner", group = "Test")
public class ServoMixerTuner extends LinearOpMode {
    private Servo ServoHood;
    private double servoPosition = 0.0317; // Initial position from Shooter

    @Override
    public void runOpMode() {
        ServoHood = hardwareMap.get(Servo.class, "ServoMixer1");
        ServoHood.setDirection(Servo.Direction.FORWARD);

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("  DPad Up/Down    → ±0.001 (fine)");
        telemetry.addLine("  DPad Left/Right → ±0.01 (coarse)");
        telemetry.addLine();
        telemetry.addData("Status", "Press START when ready");
        telemetry.update();

        waitForStart();

        ServoHood.setPosition(servoPosition);

        while (opModeIsActive()) {
            boolean changed = false;

            // Fine adjustment (±0.001)
            if (gamepad1.dpad_up) {
                servoPosition += 0.001;
                changed = true;
                sleep(100);
            } else if (gamepad1.dpad_down) {
                servoPosition -= 0.001;
                changed = true;
                sleep(100);
            }

            // Coarse adjustment (±0.01)
            if (gamepad1.dpad_right) {
                servoPosition += 0.01;
                changed = true;
                sleep(100);
            } else if (gamepad1.dpad_left) {
                servoPosition -= 0.01;
                changed = true;
                sleep(100);
            }


            // Clamp to valid servo range [0.0, 1.0]
            servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

            // Apply position to servo
            if (changed) {
                ServoHood.setPosition(servoPosition);
            }

            // Display telemetry
            telemetry.addLine("═══════════════════════════════");
            telemetry.addData("⚙ Current Position", String.format("%.4f", servoPosition));
            telemetry.addLine("───────────────────────────────");
            telemetry.update();
        }
    }
}

