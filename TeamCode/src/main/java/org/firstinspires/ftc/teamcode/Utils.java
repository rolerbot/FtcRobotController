package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

enum Color{None, Purple, Green};
public class Utils
{
    public static void GasirePozitii1(ButtonReader x, ButtonReader y, Servo Test, Servo test)
    {
        x.readValue();
        y.readValue();
        double pozitie = Test.getPosition();
        double pozitie2 = test.getPosition();
        if (x.wasJustPressed())
        {
            test.setPosition(pozitie2 + 0.01);
            Test.setPosition(pozitie + 0.01);
        }
        if (y.wasJustPressed())
        {
            test.setPosition(pozitie - 0.002);
            Test.setPosition(pozitie - 0.002);
        }
    }

    public static void GasirePozitii(ButtonReader x, ButtonReader y, Servo Test)
    {
        x.readValue();
        y.readValue();
        double pozitie = Test.getPosition();

        if (x.wasJustPressed())
        {
            Test.setPosition(pozitie + 0.001);
        }
        if (y.wasJustPressed())
        {
            Test.setPosition(pozitie - 0.001);
        }
    }
}