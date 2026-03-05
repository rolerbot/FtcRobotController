package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp
public class ShooterTuning extends OpMode {

    private DcMotorEx shooterMotor1, motorridicarebila;
    private Servo hoodServo;
    private LimeLight limelight;
    // private Drivetrain drivetrain;
    private double currenttargetVelocity = 1100;
    double F = 0;
    double P = 0;

    private final double TRANSFER_P = 10.0;
    private final double TRANSFER_I = 3.0;
    private final double TRANSFER_D = 0.0;
    private final double TRANSFER_F = 12.0;
    private final double TRANSFER_VELOCITY = 2800;

    double[] stepsizes = { 10.0, 1.0, 0.1, 0.01, 0.001 };
    int stepIndex = 1;
    ButtonReader stepIncrease, Fincrease, Fdescrease, Pincrease, Pdecrease,
            HoodUp, HoodDown, VelocityUp, VelocityDown;
    GamepadEx ct1, ct2;

    @Override
    public void init() {
        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "MotorAruncare");
        motorridicarebila = hardwareMap.get(DcMotorEx.class, "MotorRidicareBila");
        hoodServo = hardwareMap.get(Servo.class, "ServoHood");

        motorridicarebila.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorridicarebila.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorridicarebila.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients transferPIDF = new PIDFCoefficients(TRANSFER_P, TRANSFER_I, TRANSFER_D, TRANSFER_F);
        motorridicarebila.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, transferPIDF);

        shooterMotor1.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hoodServo.setDirection(Servo.Direction.FORWARD);
        hoodServo.setPosition(0.5);

        // drivetrain = new Drivetrain(ct1, null);
        // drivetrain.Initialize(hardwareMap);

        limelight = new LimeLight(true, false);
        limelight.Initialize(hardwareMap);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Gamepad1 - PIDF tuning
        stepIncrease = new ButtonReader(ct1, GamepadKeys.Button.B);
        Fincrease = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        Fdescrease = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        Pincrease = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        Pdecrease = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);

        HoodUp = new ButtonReader(ct1, GamepadKeys.Button.RIGHT_BUMPER);
        HoodDown = new ButtonReader(ct1, GamepadKeys.Button.LEFT_BUMPER);

        // Gamepad2 - Velocity control
        VelocityUp = new ButtonReader(ct2, GamepadKeys.Button.Y);
        VelocityDown = new ButtonReader(ct2, GamepadKeys.Button.A);

        telemetry.addLine("Initialized");
    }

    @Override
    public void loop() {
        // set velocity
        shooterMotor1.setVelocity(currenttargetVelocity);
        motorridicarebila.setVelocity(TRANSFER_VELOCITY);

        stepIncrease.readValue();
        Fincrease.readValue();
        Fdescrease.readValue();
        Pincrease.readValue();
        Pdecrease.readValue();
        HoodUp.readValue();
        HoodDown.readValue();
        VelocityUp.readValue();
        VelocityDown.readValue();

        limelight.Run();
        // drivetrain.Run();

        if (stepIncrease.wasJustPressed())
            stepIndex = (stepIndex + 1) % stepsizes.length;

        if (Fincrease.wasJustPressed())
            F += stepsizes[stepIndex];

        if (Fdescrease.wasJustPressed())
            F -= stepsizes[stepIndex];

        if (Pincrease.wasJustPressed())
            P += stepsizes[stepIndex];

        if (Pdecrease.wasJustPressed())
            P -= stepsizes[stepIndex];

        if (HoodUp.wasJustPressed()) {
            double newPos = hoodServo.getPosition() + 0.003;
            if (newPos <= 1.0)
                hoodServo.setPosition(newPos);
        }

        if (HoodDown.wasJustPressed()) {
            double newPos = hoodServo.getPosition() - 0.003;
            if (newPos >= 0.0)
                hoodServo.setPosition(newPos);
        }

        // Velocity control with gamepad2 Y/A buttons
        if (VelocityUp.wasJustPressed()) {
            currenttargetVelocity += 50; // Increase by 50 RPM
            currenttargetVelocity = Math.min(5000, currenttargetVelocity); // Max 5000 RPM
        }

        if (VelocityDown.wasJustPressed()) {
            currenttargetVelocity -= 50; // Decrease by 50 RPM
            currenttargetVelocity = Math.max(0, currenttargetVelocity); // Min 0 RPM
        }

        // set new PIDF values
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        double curVel1 = shooterMotor1.getVelocity();
        double error = currenttargetVelocity - curVel1;

        // telemetry
        telemetry.addLine("=== SHOOTER TUNING ===");
        telemetry.addData("Target Velocity", "%.0f RPM", currenttargetVelocity);
        telemetry.addData("Motor 1 Velocity", "%.0f RPM", curVel1);
        telemetry.addData("Error", "%.0f RPM", error);
        telemetry.addData("Error %", "%.1f%%", (error / currenttargetVelocity) * 100);
        telemetry.addLine();
        telemetry.addLine("=== TRANSFER TUNING ===");
        telemetry.addData("Transfer Target", "%.0f RPM", TRANSFER_VELOCITY);
        telemetry.addData("Transfer Velocity", "%.0f RPM", motorridicarebila.getVelocity());
        telemetry.addLine();
        telemetry.addLine("=== GAMEPAD 1 - PIDF TUNING ===");
        telemetry.addData("  F (D-Pad ←/→)", "%.4f", F);
        telemetry.addData("  P (D-Pad ↑/↓)", "%.4f", P);
        telemetry.addData("  Step (B)", "%.3f", stepsizes[stepIndex]);
        telemetry.addData("  Hood (Bumpers)", "%.3f", hoodServo.getPosition());
        telemetry.addLine();
        telemetry.addLine("=== GAMEPAD 2 - VELOCITY ===");
        telemetry.addLine("  Y: +50 RPM  |  A: -50 RPM");
        telemetry.addLine();
        telemetry.addData("Distance", "%.2f m", limelight.GetDistanceToTargetPython());
        telemetry.update();

    }

}