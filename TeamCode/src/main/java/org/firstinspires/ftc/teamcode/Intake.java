package org.firstinspires.ftc.teamcode;

import android.widget.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Subsystem {
    private DcMotorEx MotorIntake;
    private Button Forward;
    private Button Backward;
    private Button Stop;
    private GamepadEx ct1;

    private Intake(GamepadEx ct)
    {
        this.ct1 = ct;
    }
    public void LinkComponents(HardwareMap hwMap) {
        MotorIntake = hwMap.get(DcMotorEx.class, "MotorIntake");
    }

    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);
        MotorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorIntake.setDirection(DcMotor.Direction.FORWARD);
        MotorIntake.setPower(0);
    }

    public void Apas()
    {
        Forward.readValue
    }

    public void Run()
    {

    }
}
