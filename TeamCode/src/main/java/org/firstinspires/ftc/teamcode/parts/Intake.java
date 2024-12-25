package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private Servo BazaDreapta, BazaStanga;

    public Intake(HardwareMap hardwareMap){
        BazaDreapta = hardwareMap.get(Servo.class, "BazaDreapta");
        BazaStanga = hardwareMap.get(Servo.class, "BazaStanga");
    }

    public void SliderBaza(Gamepad gamepad1)
    {
        double Controler = 0.005;
        if (gamepad1.right_stick_y > Controler && BazaDreapta.getPosition() < 0.32 ||
                gamepad1.right_stick_y < -Controler && BazaStanga.getPosition() > 0.07)
        {
            BazaDreapta.setPosition(BazaDreapta.getPosition() + 0.0013 * gamepad1.right_stick_y);
            BazaStanga.setPosition(BazaStanga.getPosition() + 0.0013 * gamepad1.right_stick_y); //0.0025
        }
    }
}
