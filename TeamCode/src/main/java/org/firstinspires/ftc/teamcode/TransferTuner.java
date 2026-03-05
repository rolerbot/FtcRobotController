package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Transfer PIDF Tuner", group = "Tuning")
public class TransferTuner extends LinearOpMode {

    private DcMotorEx shooterMotor;
    private DcMotorEx transferMotor;
    private Servo hoodServo;

    // Default PIDF for a typical 435 RPM or 1150 RPM motor
    // We'll start with something very safe
    private double p = 10.0;
    private double i = 3.0;
    private double d = 0.0;
    private double f = 12.0;

    private double targetTransferVelocity = 2800;
    private boolean motorRunning = false;
    private String selectedCoefficient = "P";

    @Override
    public void runOpMode() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "MotorAruncare");
        transferMotor = hardwareMap.get(DcMotorEx.class, "MotorRidicareBila");
        hoodServo = hardwareMap.get(Servo.class, "ServoHood");

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Transfer PIDF Tuner Initialized");
        telemetry.addLine("Use D-Pad Up/Down to select P, I, D, or F");
        telemetry.addLine("Use D-Pad Left/Right to adjust values");
        telemetry.addLine("Press A to toggle Transfer Motor");
        telemetry.update();

        waitForStart();

        // Start shooter and set hood at start
        shooterMotor.setVelocity(1500);
        hoodServo.setPosition(0.51);

        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastLeft = false;
        boolean lastRight = false;
        boolean lastA = false;

        while (opModeIsActive()) {
            // --- Input Logic ---
            if (gamepad1.dpad_up && !lastUp) {
                switch (selectedCoefficient) {
                    case "P":
                        selectedCoefficient = "F";
                        break;
                    case "I":
                        selectedCoefficient = "P";
                        break;
                    case "D":
                        selectedCoefficient = "I";
                        break;
                    case "F":
                        selectedCoefficient = "D";
                        break;
                }
            }
            if (gamepad1.dpad_down && !lastDown) {
                switch (selectedCoefficient) {
                    case "P":
                        selectedCoefficient = "I";
                        break;
                    case "I":
                        selectedCoefficient = "D";
                        break;
                    case "D":
                        selectedCoefficient = "F";
                        break;
                    case "F":
                        selectedCoefficient = "P";
                        break;
                }
            }

            double change = 0.1;
            if (gamepad1.right_bumper)
                change = 1.0;
            if (gamepad1.left_bumper)
                change = 0.01;

            if (gamepad1.dpad_right && !lastRight) {
                if (selectedCoefficient.equals("P"))
                    p += change;
                else if (selectedCoefficient.equals("I"))
                    i += change;
                else if (selectedCoefficient.equals("D"))
                    d += change;
                else if (selectedCoefficient.equals("F"))
                    f += change;
            }
            if (gamepad1.dpad_left && !lastLeft) {
                if (selectedCoefficient.equals("P"))
                    p -= change;
                else if (selectedCoefficient.equals("I"))
                    i -= change;
                else if (selectedCoefficient.equals("D"))
                    d -= change;
                else if (selectedCoefficient.equals("F"))
                    f -= change;
            }

            if (gamepad1.a && !lastA) {
                motorRunning = !motorRunning;
            }

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastA = gamepad1.a;

            // --- Apply Coefficients ---
            PIDFCoefficients coeff = new PIDFCoefficients(p, i, d, f);
            transferMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeff);

            if (motorRunning) {
                transferMotor.setVelocity(targetTransferVelocity);
            } else {
                transferMotor.setVelocity(0);
            }

            // --- Telemetry ---
            telemetry.addData("Selected", selectedCoefficient);
            telemetry.addData("P", "%.4f %s", p, selectedCoefficient.equals("P") ? "<--" : "");
            telemetry.addData("I", "%.4f %s", i, selectedCoefficient.equals("I") ? "<--" : "");
            telemetry.addData("D", "%.4f %s", d, selectedCoefficient.equals("D") ? "<--" : "");
            telemetry.addData("F", "%.4f %s", f, selectedCoefficient.equals("F") ? "<--" : "");
            telemetry.addLine("───────────────────────────────");
            telemetry.addData("Target Velocity", targetTransferVelocity);
            telemetry.addData("Actual Velocity", transferMotor.getVelocity());
            telemetry.addData("Error", targetTransferVelocity - transferMotor.getVelocity());
            telemetry.addData("Motor State", motorRunning ? "RUNNING" : "STOPPED");
            telemetry.addLine("───────────────────────────────");
            telemetry.addData("Shooter Vel", shooterMotor.getVelocity());
            telemetry.addData("Hood Pos", hoodServo.getPosition());
            telemetry.update();
        }
    }
}
