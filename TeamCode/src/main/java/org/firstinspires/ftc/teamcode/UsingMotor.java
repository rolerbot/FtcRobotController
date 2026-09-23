package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "UsingMotor", group = "Linear Opmode")
public class UsingMotor extends LinearOpMode {
    private GamepadEx ct1;
    private DcMotorEx motor;

    private void MapControlerButtons() {
        ct1 = new GamepadEx(gamepad1);
    }

    private void Initialize() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);
        MapControlerButtons();
    }

    // This method changes the power of the motor when the D-Pad buttons are pressed
    private void PowerChange()
    {
        ButtonReader forwardButton = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        ButtonReader backwardButton = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
        ButtonReader stopButton = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);

        forwardButton.readValue();
        backwardButton.readValue();
        stopButton.readValue();

        if (forwardButton.wasJustPressed()) {
            motor.setPower(0.3);
        }

        if (backwardButton.wasJustPressed()) {
            motor.setPower(-0.3);
        }

        if (stopButton.wasJustPressed()) {
            motor.setPower(0);
        }
    }

    // This method changes the velocity of the motor when the A or B buttons are pressed
    private void VelocityChange()
    {
        ButtonReader increaseVelocityButton = new ButtonReader(ct1, GamepadKeys.Button.A);
        ButtonReader decreaseVelocityButton = new ButtonReader(ct1, GamepadKeys.Button.B);

        increaseVelocityButton.readValue();
        decreaseVelocityButton.readValue();

        if (increaseVelocityButton.wasJustPressed()) {
            motor.setVelocity(motor.getVelocity() + 100);
        }

        if (decreaseVelocityButton.wasJustPressed()) {
            motor.setVelocity(motor.getVelocity() - 100);
        }
    }

    public void runOpMode() {
        Initialize();
        waitForStart();

        while (opModeIsActive()) {
            PowerChange();
            VelocityChange();
            // Display telemetry data for motor power, velocity, and position
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Motor Velocity", motor.getVelocity());
            telemetry.addData("Motor Position", motor.getCurrentPosition());
            telemetry.update();
        }
        telemetry.update();
    }
}
