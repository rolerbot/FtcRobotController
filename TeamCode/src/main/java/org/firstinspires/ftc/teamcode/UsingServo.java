package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "UsingServo", group = "Linear Opmode")
public class UsingServo extends LinearOpMode {
    private GamepadEx ct1;
    private Servo servo;

    private void MapControlerButtons() {
        ct1 = new GamepadEx(gamepad1);
    }

    private void Initialize() {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(0);
    }

    // This method changes the position of the servo when the D-Pad buttons are pressed
    // Gasire poszitii
    private void PositionChange()
    {
        ButtonReader forwardButton = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        ButtonReader backwardButton = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
        ButtonReader stopButton = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);

        forwardButton.readValue();
        backwardButton.readValue();
        stopButton.readValue();

        if (forwardButton.wasJustPressed()) {
            servo.setPosition(servo.getPosition() + 0.1);
        }

        if (backwardButton.wasJustPressed()) {
            servo.setPosition(servo.getPosition() - 0.1);
        }

        if (stopButton.wasJustPressed()) {
            servo.setPosition(0);
        }
    }

    // This method changes the position of the servo based on the left stick Y-axis value
    private void ControllerPositionChange()
    {
        double leftStickY = ct1.getLeftY();
        if (Math.abs(leftStickY) > 0.1)
            servo.setPosition(leftStickY);
        // Intre -1 si 1
    }

    public void runOpMode() {
        Initialize();
        waitForStart();

        while (opModeIsActive()) {
            PositionChange();
            // Display telemetry data for servo position
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
        telemetry.update();
    }
}
