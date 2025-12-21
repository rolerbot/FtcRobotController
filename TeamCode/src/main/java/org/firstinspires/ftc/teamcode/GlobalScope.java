package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class GlobalScope extends LinearOpMode {
    public DcMotorEx MotorFS = null;
    /// Fata stanga
    public DcMotorEx MotorFD = null;
    /// Fata dreapta
    public DcMotorEx MotorSS = null;
    /// Spate stanga
    public DcMotorEx MotorSD = null;
    /// Spate dreapta
    public DcMotorEx MotorIN = null;

    void LinkComponents()
    {
        MotorFS = hardwareMap.get(DcMotorEx.class, "MotorFS");
        MotorFD = hardwareMap.get(DcMotorEx.class, "MotorFD");
        MotorSS = hardwareMap.get(DcMotorEx.class, "MotorSS");
        MotorSD = hardwareMap.get(DcMotorEx.class, "MotorSD");
        MotorIN = hardwareMap.get(DcMotorEx.class, "MotorIN");
    }

    void Initialise()
    {
        LinkComponents();

        //---------------------ROTZI---------------

        MotorIN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //--------------------------SLIDE-------------


        //------------------------SERVO---------------------

    }
    void MapControlerButtons() {
        InitComponente();

        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);

        Viteza = new ButtonReader(ct1, GamepadKeys.Button.B);
        ButtonSus = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        ButtonSus = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
    }

    void InitComponente()
    {


    }

    void Controler()
    {
        InitComponente();

        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);

        Viteza = new ButtonReader(ct1, GamepadKeys.Button.B);

    }

    /// TELEOP

    double drive, strafe, twist;
    double[] speeds = new double[4];
    double schimbator = 0.4;//Viteza 0.4
    GamepadEx ct1, ct2;
    ButtonReader Viteza;
    /// cautator de viteze
    ButtonReader SliderSus, SliderJos;
    ButtonReader ButtonSus, ButtonJos;

    void MiscareBaza()
    {
        Viteza.readValue();
        if (Viteza.wasJustPressed()) {
            schimbator = 1.4 - schimbator;
            telemetry.addData("viteza este", schimbator);
            telemetry.update();
        }
        drive = -gamepad1.left_stick_y * schimbator;
        strafe = gamepad1.left_stick_x * schimbator;
        twist = schimbator * (gamepad1.right_trigger - gamepad1.left_trigger);
        speeds[0] = (drive + strafe + twist);//FS
        speeds[1] = (drive - strafe - twist);//FD
        speeds[2] = (drive - strafe + twist);//SS
        speeds[3] = (drive + strafe - twist);//SD
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
        MotorFS.setPower(speeds[0]);
        MotorFD.setPower(speeds[1]);
        MotorSS.setPower(speeds[2]);
        MotorSD.setPower(speeds[3]);
    }
    void MotorIn()
    {
    if(ButtonJos.wasJustPressed())
    {
        MotorIN.setPower(-1);
    }
    else if(ButtonSus.wasJustPressed())
    {
        MotorIN.setPower(1);
    }
    else MotorFS.setPower(0);
    }
    void GasirePozitii(ButtonReader x, ButtonReader y, Servo Test, Servo test)
    {
        x.readValue();
        y.readValue();
        double pozitie = Test.getPosition();
        double pozitie2 = test.getPosition();
        if(x.wasJustPressed())
        {
            test.setPosition(pozitie2 + 0.001);
            Test.setPosition(pozitie + 0.001);
        }
        if(y.wasJustPressed())
        {
            test.setPosition(pozitie - 0.001);
            Test.setPosition(pozitie - 0.001);
        }
    }

}
