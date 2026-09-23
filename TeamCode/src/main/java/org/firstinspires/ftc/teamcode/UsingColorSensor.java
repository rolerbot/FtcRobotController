package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "UsingColorSensor", group = "Linear Opmode")
public class UsingColorSensor extends LinearOpMode {
    private ColorSensor colorSensor;

    private void Initialize() {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    private void DisplayColorSensorData() {
        // Display telemetry data for color sensor readings
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.addData("ARGB", colorSensor.argb());
    }

    public void runOpMode() {
        Initialize();
        waitForStart();

        while (opModeIsActive()) {
            DisplayColorSensorData();
            telemetry.update();
        }
        telemetry.update();
    }
}

