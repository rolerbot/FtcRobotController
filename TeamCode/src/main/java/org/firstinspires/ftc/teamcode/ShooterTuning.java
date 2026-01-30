package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class ShooterTuning extends OpMode {

    private DcMotorEx shooterMotor1, shooterMotor2;

    // PIDF values tuned for your measured max velocity
    private double F = 15.16;  // F = 32767 / max velocity (740 ticks/sec)
    private double P = 0.002; // small P to help low-speed shots
    private double I = 0.0;
    private double D = 0.0;

    // Velocities to test (all ≤ max physical velocity)
    private double testVelocities = 1100;
    private int currentIndex = 0;

    // Gamepad buttons
    private ButtonReader nextVelocity, prevVelocity;
    private GamepadEx ct1;

    @Override
    public void init() {
        ct1 = new GamepadEx(gamepad1);

        shooterMotor1  = hardwareMap.get(DcMotorEx.class, "MotorAruncare1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "MotorAruncare2");

        shooterMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        shooterMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Buttons to cycle velocities
        nextVelocity = new ButtonReader(ct1, GamepadKeys.Button.B);
        prevVelocity = new ButtonReader(ct1, GamepadKeys.Button.X);

        telemetry.addLine("Initialized. Use B/X to change test velocity.");
    }

    @Override
    public void loop() {
        // Read buttons
        nextVelocity.readValue();
        prevVelocity.readValue();

        if (nextVelocity.wasJustPressed())
            testVelocities -= 10;
        if (prevVelocity.wasJustPressed())
            testVelocities += 10;

        double currentTarget = testVelocities;

        // Set motors to target velocity
        shooterMotor1.setVelocity(currentTarget);
        shooterMotor2.setVelocity(currentTarget);

        // Read actual velocity
        double curVel1 = shooterMotor1.getVelocity();
        double curVel2 = shooterMotor2.getVelocity();
        double error1 = currentTarget - curVel1;
        double error2 = currentTarget - curVel2;

        // Telemetry
        telemetry.addData("Test Index", currentIndex);
        telemetry.addData("Target Velocity", currentTarget);
        telemetry.addData("Motor1 Velocity", curVel1);
        telemetry.addData("Motor2 Velocity", curVel2);
        telemetry.addData("Motor1 Error", error1);
        telemetry.addData("Motor2 Error", error2);
        telemetry.addData("PIDF", "P: %.4f  I: %.4f  D: %.4f  F: %.4f", P, I, D, F);
        telemetry.update();
    }
}