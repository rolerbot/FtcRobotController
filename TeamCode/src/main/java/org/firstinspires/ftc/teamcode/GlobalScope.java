package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class GlobalScope extends LinearOpMode
{
    public DcMotorEx MotorFS = null; /// Fata stanga
    public DcMotorEx MotorFD = null; /// Fata dreapta
    public DcMotorEx MotorSS = null; /// Spate stanga
    public DcMotorEx MotorSD = null; /// Spate dreapta
    public DcMotorEx Slider = null;
    public Servo ServoRotire = null;
    public Servo BazaDreapta = null;
    public Servo BazaStanga = null;
    public Servo IntakeStanga = null;
    public Servo IntakeDreapta = null;
    public Servo OutakeStanga = null;
    public Servo OutakeDreapta = null;
    public Servo ServoGhearaIntake = null; //Cleste Stanga
    public Servo ServoGhearaOutake = null; //Cleste Dreapta

    void LinkComponents() {
        MotorFS = hardwareMap.get(DcMotorEx.class, "MotorFS");
        MotorFD = hardwareMap.get(DcMotorEx.class, "MotorFD");
        MotorSS = hardwareMap.get(DcMotorEx.class, "MotorSS");
        MotorSD = hardwareMap.get(DcMotorEx.class, "MotorSD");
        Slider = hardwareMap.get(DcMotorEx.class, "Slider");
        ServoGhearaOutake = hardwareMap.get(Servo.class, "ServoGhearaOutake");
        ServoGhearaIntake = hardwareMap.get(Servo.class, "ServoGhearaIntake");
        ServoRotire = hardwareMap.get(Servo.class, "ServoRotire");
        IntakeDreapta = hardwareMap.get(Servo.class, "IntakeDreapta");
        IntakeStanga = hardwareMap.get(Servo.class, "IntakeStanga");
        BazaDreapta = hardwareMap.get(Servo.class, "BazaDreapta");
        BazaStanga = hardwareMap. get(Servo.class, "BazaStanga");
        OutakeStanga = hardwareMap.get(Servo.class, "OutakeStanga");
        OutakeDreapta = hardwareMap.get(Servo.class, "OutakeDreapta");
    }

    void Initialise() {
        LinkComponents();
        //---------------------ROTZI---------------
        MotorFS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFS.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorSS.setDirection(DcMotorSimple.Direction.REVERSE);

        //--------------------------SLIDE-------------
        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slider.setDirection(DcMotorSimple.Direction.REVERSE);//Reverse

        //------------------------SERVO---------------------
        ServoGhearaIntake.setDirection(Servo.Direction.FORWARD);
        ServoGhearaOutake.setDirection(Servo.Direction.REVERSE);
        ServoRotire.setDirection(Servo.Direction.FORWARD);
        ServoRotire.scaleRange(0,0.2);
        BazaDreapta.setDirection(Servo.Direction.FORWARD);
        BazaStanga.setDirection(Servo.Direction.REVERSE);
        IntakeStanga.setDirection(Servo.Direction.FORWARD);
        IntakeDreapta.setDirection(Servo.Direction.REVERSE);
        OutakeStanga.setDirection(Servo.Direction.REVERSE);
        OutakeDreapta.setDirection(Servo.Direction.FORWARD);
    }

    /// TELEOP
    double drive;
    double strafe;
    double twist;
    double[] speeds = new double[4];
    double schimbator = 0.4;
    int cnt = 8000;
    double vit = 1; //Viteza
    int cleste1 = 0, cleste2 = 0, pozitieIntake = 0;
    double PozBrat[] = {0, 0.1, 0.2, 0.3};
    GamepadEx ct1, ct2;
    ButtonReader Viteza; /// cautator de viteze
    ButtonReader RotireStanga, RotireDreapta;
    ButtonReader IntakeSus, IntakeJos;
    TriggerReader GhearaIntake, GhearaOutake;
    ButtonReader OutakeJos, OutakeSus;

    void MiscareBaza()
    {
        Viteza.readValue();
        if(Viteza.wasJustPressed())
        {
            schimbator = 1.4 - schimbator;
            telemetry.addData("viteza este", schimbator);
            telemetry.update();
        }
        drive  = -gamepad1.left_stick_y * schimbator;
        strafe = gamepad1.left_stick_x * schimbator;
        twist  = schimbator*(gamepad1.right_trigger-gamepad1.left_trigger) ;
        speeds[0]=(drive + strafe + twist);//FS
        speeds[1]=(drive - strafe - twist);//FD
        speeds[2]=(drive - strafe + twist);//SS
        speeds[3]=(drive + strafe - twist);//SD
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
        MotorFS.setPower(speeds[0]);
        MotorFD.setPower(speeds[1]);
        MotorSS.setPower(speeds[2]);
        MotorSD.setPower(speeds[3]);
    }

    void SliderExtend()
    {
        double deadzone = 0.5;
        if (gamepad2.left_stick_y > deadzone && Slider.getCurrentPosition() > cnt) // Coboara
            Slider.setPower(-vit);
        else if (gamepad2.left_stick_y < -deadzone && Slider.getCurrentPosition() < cnt) //Urca
            Slider.setPower(vit);
        else
            Slider.setPower(0);
        /*if(ok == 0)
        {
            Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ok = 1;
        }*/
    }



    void SliderBaza()
    {
        double deadzone = 0.005;
        if (gamepad2.left_stick_x > deadzone && BazaDreapta.getPosition() < 0.33 ||
            gamepad2.left_stick_x < -deadzone && BazaStanga.getPosition() > 0.007)
        {
            BazaDreapta.setPosition(BazaDreapta.getPosition() + 0.003 * gamepad2.left_stick_x);
            BazaStanga.setPosition(BazaStanga.getPosition() + 0.003 * gamepad2.left_stick_x);
        }
    }

    /**void SliderBaza()
    {
        if (gamepad2.left_stick_x > 0.005 && BazaDreapta.getPosition() < 0.075) // Extinde
        {
           BazaDreapta.setPosition(BazaDreapta.getPosition() + 0.0003 * gamepad2.left_stick_x);
           BazaStanga.setPosition(BazaStanga.getPosition() + 0.0003 * gamepad2.left_stick_x);
           //BazaDreapta.setPosition(0.07);
           //BazaStanga.setPosition(0.07);
        }
        else if (gamepad2.left_stick_x < -0.005 && BazaStanga.getPosition() > 0.018) //Retrage
        {
            BazaDreapta.setPosition(BazaDreapta.getPosition() + 0.0003 * gamepad2.left_stick_x);
            BazaStanga.setPosition(BazaStanga.getPosition() + 0.0003 * gamepad2.left_stick_x);
            //BazaDreapta.setPosition(0.02);
            //BazaStanga.setPosition(0.02);
        }
    }*/

    void Cleste()
    {
        GhearaIntake.readValue();
        GhearaOutake.readValue();

        if(GhearaIntake.wasJustPressed() && cleste1 == 0)
        {
            cleste1++;
            ServoGhearaIntake.setPosition(0.022);
        }
        else if(GhearaIntake.wasJustPressed() && cleste1 == 1)
        {
            cleste1--;
            ServoGhearaIntake.setPosition(0); ///0.02
        }

        if(GhearaOutake.wasJustPressed() && cleste2 == 0)
        {
            cleste2++;
            ServoGhearaOutake.setPosition(0.022);
        }
        else if(GhearaOutake.wasJustPressed() && cleste2 == 1)
        {
            cleste2--;
            ServoGhearaOutake.setPosition(0); ///0.02
        }
    }

    void Intake()
    {
        IntakeSus.readValue();
        IntakeJos.readValue();

        if(IntakeSus.wasJustPressed())
        {
              IntakeStanga.setPosition(1);
              IntakeDreapta.setPosition(1);
              //pozitieIntake++;
        }
        if(IntakeJos.wasJustPressed())
        {
            IntakeDreapta.setPosition(0);
            IntakeStanga.setPosition(0);
            //pozitieIntake--;
        }
    }

    void Outake()
    {
        OutakeSus.readValue();
        OutakeJos.readValue();

        if(OutakeSus.wasJustPressed())
        {
            OutakeStanga.setPosition(0.5);
            OutakeDreapta.setPosition(0.5);
        }
        if(OutakeJos.wasJustPressed())
        {
            OutakeDreapta.setPosition(0.6);
            OutakeStanga.setPosition(0.55);
        }
    }

    void Roteste()
    {
        RotireStanga.readValue();
        RotireDreapta.readValue();
        double PosInitial = ServoRotire.getPosition();
        if(RotireStanga.wasJustPressed())
            ServoRotire.setPosition(PosInitial - 0.125);
        if(RotireDreapta.wasJustPressed())
            ServoRotire.setPosition(PosInitial + 0.125);
    }
}
