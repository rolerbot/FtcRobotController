package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.qualcomm.robotcore.hardware.Servo;

enum Color
{
    None,
    Purple,
    Green
};

enum ShootingState
{
    Arranged,
    Green,
    Purple,
    Unarranged,
    None
};

enum AlignmentState
{
    IDLE,
    ROTATING,
    ADJUSTING_DISTANCE,
    COMPLETED
}

public class Utils
{
    public static String ColorToString(Color color)
    {
        if (color == Color.None) return "None";
        if (color == Color.Green) return "Green";
        return "Purple";
    }

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
            Test.setPosition(pozitie + 0.5);
        }
        if (y.wasJustPressed())
        {
            Test.setPosition(pozitie - 0.5);
        }
    }
}