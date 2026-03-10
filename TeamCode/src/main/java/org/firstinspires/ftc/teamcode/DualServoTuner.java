package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mixer enc and servo positions", group = "Tuning")
public class DualServoTuner extends OpMode
{
    private Servo servo1, servo2;
    private DcMotorEx MotorMixer;
    private double offsetPosition = 0.09028;
    private double pos = 0.0827 + 2 * offsetPosition;
    private double step = 0.01;
    private boolean lastUp, lastDown, lastLB, lastRB;

    @Override
    public void init() {

        servo1 = hardwareMap.get(Servo.class, "ServoMixer1");
        servo2 = hardwareMap.get(Servo.class, "ServoMixer2");
        MotorMixer = hardwareMap.get(DcMotorEx.class, "MotorIN");

        servo1.setPosition(pos);
        servo2.setPosition(pos);

        MotorMixer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorMixer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorMixer.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();

            // Step adjustment
            if (gamepad1.left_bumper && !lastLB)
                step /= 10.0;
            if (gamepad1.right_bumper && !lastRB)
                step *= 10.0;

            step = Math.max(0.0001, Math.min(0.1, step));

            // Servo (Mixer 1)
            if (gamepad1.dpad_left && !lastUp)
                pos += step;
            if (gamepad1.dpad_right && !lastDown)
                pos -= step;

            pos = Math.max(0, Math.min(1, pos));

            servo1.setPosition(pos);
            servo2.setPosition(pos);

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;

            telemetry.addData("TUNER", "--- Controls ---");
            telemetry.addData("Step (LB/RB)", "%.4f", step);
            telemetry.addData("Servo 2 (Y/A)", "%.4f", pos);
            telemetry.addData("Motor Position", MotorMixer.getCurrentPosition());
            telemetry.update();
    }
}
