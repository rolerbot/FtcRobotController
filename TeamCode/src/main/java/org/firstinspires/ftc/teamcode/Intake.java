package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Subsystem{
    private GamepadEx ct1;
    public DcMotorEx MotorIN = null;
    ButtonReader ButtonSus, ButtonJos;
    private boolean isReversed = false, isForward = false;
    private double motorPower = 0.8;

    public Intake(GamepadEx ct1){
        this.ct1 = ct1;
    }
    public void LinkComponents(HardwareMap hwMap){
        MotorIN = hwMap.get(DcMotorEx.class, "MotorIN");
    }
    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);

        ButtonSus = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        ButtonJos = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);

        MotorIN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorIN.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    private void ReadButtons(){
        ButtonSus.readValue();
        ButtonJos.readValue();
    }
    private void MotorIntake()
    {
        if (ButtonSus.wasJustPressed() && !isForward)
        {
            isForward = true;
            isReversed = false;
            MotorIN.setPower(motorPower);
        }
        else if(ButtonSus.wasJustPressed() && isForward)
            SetMotorPower(0);
    }
    private void MotorIntakeReverse()
    {
        if (ButtonJos.wasJustPressed() && !isReversed)
        {
            isForward = false;
            isReversed = true;
            MotorIN.setPower(-motorPower);
        }
        else if (ButtonJos.wasJustPressed() && isReversed)
            SetMotorPower(0);
    }

    protected void SetMotorPower(double pow)
    {
        if(pow > 1 || pow < -1)
            return;
        MotorIN.setPower(pow);
        isReversed = false;
        isForward = false;
    }
    public boolean IsStopped(){
        return (!isForward  && !isReversed);
    }
    public void Run()
    {
        ReadButtons();
        MotorIntake();
        MotorIntakeReverse();
    }
}
