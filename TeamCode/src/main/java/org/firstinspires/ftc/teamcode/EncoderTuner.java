package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Encoder Debugger", group = "Debug")
public class EncoderTuner extends LinearOpMode {
    private DcMotorEx motor;

    @Override
    public void runOpMode() {
        // Based on Mixer.java, the mixer motor is "MotorIN"
        motor = hardwareMap.get(DcMotorEx.class, "MotorTurela");

        // Reset encoder at Init
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Encoder Debugger Initialized");
        telemetry.addLine("Motor Name: MotorIN");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.addLine("───────────────────────────────");
            telemetry.addLine("Move the mixer manually to see");
            telemetry.addLine("the encoder values change.");
            telemetry.update();
        }
    }
}
