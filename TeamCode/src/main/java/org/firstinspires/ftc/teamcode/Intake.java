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
    int counterInversare = 0;
    int counterRotire = 0;

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
    void MotorIntake()
    {
        ButtonSus.readValue();

        if (ButtonSus.wasJustPressed() && counterRotire == 1)
        {
            StopMotor();
        }
        else if (ButtonSus.wasJustPressed() && counterRotire == 0)
        {
            counterRotire = 1;
            MotorIN.setPower(0.65);
        }
    }
    void MotorIntakeReverse()
    {
        ButtonJos.readValue();
        if (ButtonJos.wasJustPressed() && counterInversare == 1)
        {
            StopMotor();
        }
        else if (ButtonJos.wasJustPressed() && counterInversare == 0)
        {
            //start reverse
            counterRotire = 0;
            counterInversare = 1;
            MotorIN.setPower(-0.65);
        }
    }
    public void StopMotor()
    {
        counterRotire = 0;
        counterInversare = 0;
        MotorIN.setPower(0);
    }
    public boolean IsStopped(){
        return counterRotire == 0;
    }

    public void Run()
    {
        MotorIntake();
        MotorIntakeReverse();
    }
}
