package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Turret Calibration - Reset at Center", group = "Calibration")
public class TurretCalibrationOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx motorTurret = hardwareMap.get(DcMotorEx.class, "MotorTurela");

        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorTurret.setDirection(DcMotorEx.Direction.REVERSE);
        motorTurret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("=== Turret Calibration ===");
        telemetry.addLine("");
        telemetry.addLine("Instructions:");
        telemetry.addLine("1. Use joystick to move turret to CENTER");
        telemetry.addLine("   (forward-facing position)");
        telemetry.addLine("");
        telemetry.addLine("2. Press 'A' to reset encoder to 0");
        telemetry.addLine("");
        telemetry.addLine("3. Encoder will now read:");
        telemetry.addLine("   • 0 at center");
        telemetry.addLine("   • +457 at left limit");
        telemetry.addLine("   • -457 at right limit");
        telemetry.addLine("");
        telemetry.addLine("4. Press 'B' to test rotation to limits");
        telemetry.update();

        waitForStart();

        boolean encoderReset = false;

        while (opModeIsActive()) {
            // Manual control
            double manualPower = -gamepad1.left_stick_x * 0.25;
            motorTurret.setPower(manualPower);

            // Reset encoder at center position
            if (gamepad1.a) {
                motorTurret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                motorTurret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                encoderReset = true;
            }

            int currentTicks = motorTurret.getCurrentPosition();

            telemetry.addLine("--- STATUS ---");
            telemetry.addData("Encoder", currentTicks);
            telemetry.addData("Calibrated", encoderReset ? "YES ✓" : "NO");
            telemetry.addLine("");

            if (encoderReset) {
                telemetry.addLine("✓ Calibration complete!");
                telemetry.addLine("");
                telemetry.addLine("Expected values:");
                telemetry.addData("At center", "0 ticks");
                telemetry.addData("At left limit", "+457 ticks");
                telemetry.addData("At right limit", "-457 ticks");
                telemetry.addLine("");

                // Show current position interpretation
                if (Math.abs(currentTicks) < 50) {
                    telemetry.addData("Position", "CENTER ✓");
                } else if (currentTicks > 400) {
                    telemetry.addData("Position", "LEFT LIMIT");
                } else if (currentTicks < -400) {
                    telemetry.addData("Position", "RIGHT LIMIT");
                } else if (currentTicks > 0) {
                    telemetry.addData("Position", "Left of center");
                } else {
                    telemetry.addData("Position", "Right of center");
                }
            } else {
                telemetry.addLine("Move to CENTER, then press A");
            }

            telemetry.addLine("");
            telemetry.addLine("Controls:");
            telemetry.addLine("• Left Stick X: Move turret");
            telemetry.addLine("• A: Reset encoder to 0");
            telemetry.update();
        }
    }
}
