package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Subsystem
{
    private GamepadEx ct1;
    public DcMotorEx MotorIN = null;
    ButtonReader ButtonSus, ButtonJos;
    private boolean isStarted = false;
    private double motorPower = 1.0;
    private final TelemetryCustom telemetry;

    public Intake(TelemetryCustom tl, GamepadEx ct1)
    {
        this.ct1 = ct1;
        this.telemetry = tl;
    }

    public Intake(TelemetryCustom tl) {this.telemetry = tl;};

    public void LinkComponents(HardwareMap hwMap)
    {
        MotorIN = hwMap.get(DcMotorEx.class, "MotorIN");
    }

    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);

        // Only initialize buttons if gamepad exists (teleop mode)
        if (ct1 != null)
        {
            ButtonSus = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
            ButtonJos = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
        }

        MotorIN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorIN.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void ReadButtons()
    {
        if (ButtonSus != null && ButtonJos != null)
        {
            ButtonSus.readValue();
            ButtonJos.readValue();
        }
    }

    public void SetPowerMax()
    {
        MotorIN.setPower(motorPower);
        isStarted = true;
    }

    private void MotorIntake()
    {
        if (ButtonSus.wasJustPressed() && !isStarted)
        {
           isStarted = true;
            MotorIN.setPower(motorPower);
        } else if (ButtonSus.wasJustPressed() && isStarted)
            SetMotorPower(0);
    }

    private void MotorIntakeReverse()
    {
        if (ButtonJos.wasJustPressed() && !isStarted)
        {
            isStarted = true;
            MotorIN.setPower(-motorPower);
        } else if (ButtonJos.wasJustPressed() && isStarted)
            SetMotorPower(0);
    }

    public void SetMotorPower(double pow)
    {
        if (pow > 1 || pow < -1)
            return;
        MotorIN.setPower(pow);

        // Update state flags to match actual motor state
        if (pow > 0) {
            isStarted = true;
        } else if (pow < 0) {
            isStarted = false;
        } else {
            isStarted = false;
        }
    }

    public boolean IsMoving() {return isStarted;}

    public boolean IsStopped() {return !isStarted;}

    public void Run()
    {
        ReadButtons();
        MotorIntake();
        MotorIntakeReverse();
    }
}
