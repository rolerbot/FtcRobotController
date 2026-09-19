package org.firstinspires.ftc.teamcode;

import android.widget.Button;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Subsystem {
    private DcMotorEx MotorIntake;
    ButtonReader Forward;
     ButtonReader Backward;
     ButtonReader Stop;
    private final GamepadEx ct1;

    public Intake(GamepadEx ct)
    {
        this.ct1 = ct;
    }
    public void LinkComponents(HardwareMap hwMap) {
        MotorIntake = hwMap.get(DcMotorEx.class, "MotorIntake");
    }

    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);

        Forward = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        Backward = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
        Stop = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);

        MotorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorIntake.setDirection(DcMotor.Direction.FORWARD);
        MotorIntake.setPower(0);
    }

    public void Apas() {
        Forward.readValue();
        Backward.readValue();
        Stop.readValue();

        if (Forward.wasJustPressed()) {
            MotorIntake.setPower(0.3);
        }

        if (Backward.wasJustPressed()) {
            MotorIntake.setPower(-0.3);
        }
        if (Stop.wasJustPressed()) {
            MotorIntake.setPower(0);
        }
    }
    public void Run()
    {
        Apas();
    }
}
