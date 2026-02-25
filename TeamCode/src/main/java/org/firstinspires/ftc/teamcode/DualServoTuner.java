package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Dual Servo Tuner", group = "Tuning")
public class DualServoTuner extends LinearOpMode {
    private Servo servo1, servo2;
    private double pos1 = 0.5, pos2 = 0.5;
    private double step = 0.01;
    private boolean lastUp, lastDown, lastLeft, lastRight, lastLB, lastRB;

    @Override
    public void runOpMode() {
        servo1 = hardwareMap.get(Servo.class, "ServoMixer1");
        servo2 = hardwareMap.get(Servo.class, "ServoMixer2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Step adjustment
            if (gamepad1.left_bumper && !lastLB)
                step /= 10.0;
            if (gamepad1.right_bumper && !lastRB)
                step *= 10.0;
            step = Math.max(0.0001, Math.min(0.1, step));

            // Servo 1 (Mixer 1)
            if (gamepad1.dpad_up && !lastUp)
                pos1 += step;
            if (gamepad1.dpad_down && !lastDown)
                pos1 -= step;

            // Servo 2 (Mixer 2)
            if (gamepad1.y && !lastLeft)
                pos2 += step;
            if (gamepad1.a && !lastRight)
                pos2 -= step;

            pos1 = Math.max(0, Math.min(1, pos1));
            pos2 = Math.max(0, Math.min(1, pos2));

            servo1.setPosition(pos1);
            servo2.setPosition(pos2);

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;
            lastLeft = gamepad1.y;
            lastRight = gamepad1.a;

            telemetry.addData("TUNER", "--- Controls ---");
            telemetry.addData("Step (LB/RB)", "%.4f", step);
            telemetry.addData("Servo 1 (Dpad U/D)", "%.4f", pos1);
            telemetry.addData("Servo 2 (Y/A)", "%.4f", pos2);
            telemetry.update();
        }
    }
}
