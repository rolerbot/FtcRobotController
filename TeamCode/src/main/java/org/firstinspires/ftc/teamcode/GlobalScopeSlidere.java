package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public abstract class GlobalScopeSlidere extends LinearOpMode
{
    public DcMotorEx MotorFS = null; /// Fata stanga
    public DcMotorEx MotorFD = null; /// Fata dreapta
    public DcMotorEx MotorSS = null; /// Spate stanga
    public DcMotorEx MotorSD = null; /// Spate dreapta
    public DcMotorEx Slider = null;
    public TouchSensor RevButon = null; //Buton oprire
    public Servo ServoRotire = null;
    public Servo BazaDreapta = null;
    public Servo BazaStanga = null;
    public Servo ServoDrona = null; // Servo Drona
    public Servo ServoGhearaDreapta = null; //Cleste Stanga
    public Servo ServoGhearaStanga = null; //Cleste Dreapta

    void LinkComponents() {
        MotorFS = hardwareMap.get(DcMotorEx.class, "MotorFS");
        MotorFD = hardwareMap.get(DcMotorEx.class, "MotorFD");
        MotorSS = hardwareMap.get(DcMotorEx.class, "MotorSS");
        MotorSD = hardwareMap.get(DcMotorEx.class, "MotorSD");
        Slider = hardwareMap.get(DcMotorEx.class, "Slider");
        RevButon = hardwareMap.get(TouchSensor.class, "RevButon");
        ServoDrona = hardwareMap.get(Servo.class, "SDrona");
        ServoGhearaStanga = hardwareMap.get(Servo.class, "CDreapta");
        ServoGhearaDreapta = hardwareMap.get(Servo.class, "CStanga");
        ServoRotire = hardwareMap.get(Servo.class, "SRotire");
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

        //--------------------------BRATZ-------------
        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slider.setDirection(DcMotorSimple.Direction.REVERSE);//Reverse

        //------------------------Servo---------------------
        ServoDrona.setDirection(Servo.Direction.FORWARD); //Vedem daca trebuie Reverse
        ServoGhearaDreapta.setDirection(Servo.Direction.FORWARD);
        ServoGhearaStanga.setDirection(Servo.Direction.REVERSE);
        ServoRotire.setDirection(Servo.Direction.FORWARD);
        ServoRotire.scaleRange(0,0.2);
    }

    /// TELEOP
    double drive;
    double strafe;
    double twist;
    double[] speeds = new double[4];
    double schimbator = 0.4;
    int cnt = 8000;
    int ok = 0;
    double vit = 1; //Viteza
    int cleste1 = 0, cleste2 = 0, pozitieActualaIntake = 0;
    GamepadEx ct1, ct2;
    ButtonReader IAMSPEED; /// cautator de viteze
    ButtonReader Vit, Launch;
    ButtonReader RotireStanga, RotireDreapta;
    ButtonReader BratSus, BratJos;
    ButtonReader VitezaPozitivaIntake, IntakeUp, IntakeDown;
    TriggerReader GhearaStanga, GhearaDreapta;
    ButtonReader VitezaNegativaIntake;

    void MiscareBaza()
    {
        IAMSPEED.readValue();
        if(IAMSPEED.wasJustPressed())
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
        if (gamepad2.left_stick_y > 0.5 && !RevButon.isPressed()) // Coboara
        {
            Slider.setPower(-vit); //-
            ok = 0;
        }
        else if (gamepad2.left_stick_y < -0.5 && Slider.getCurrentPosition() < cnt) //Urca
            Slider.setPower(vit);
        else
            Slider.setPower(0);
        if(RevButon.isPressed() && ok == 0)
        {
            Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ok = 1;
        }
    }

    void SliderBaza()
    {
        if (gamepad1.left_stick_y > 0.5 && BazaDreapta.getPosition() > 0) // Coboara
        {
           BazaDreapta.setPosition(BazaDreapta.getPosition() - 0.01);
           BazaStanga.setPosition(BazaStanga.getPosition() - 0.01);
        }
        else if (gamepad1.left_stick_y < -0.5 && BazaDreapta.getPosition() < 0.5)
        {
            BazaDreapta.setPosition(BazaDreapta.getPosition() + 0.01);
            BazaStanga.setPosition(BazaStanga.getPosition() + 0.01);
        }
    }

    void Cleste()
    {
        GhearaStanga.readValue();
        GhearaDreapta.readValue();

        if(GhearaStanga.wasJustPressed() && cleste1 == 0)
        {
            cleste1++;
            ServoGhearaStanga.setPosition(0);
        }
        else if(GhearaStanga.wasJustPressed() && cleste1 == 1)
        {
            cleste1--;
            ServoGhearaStanga.setPosition(0.6);
        }

        if(GhearaDreapta.wasJustPressed() && cleste2 == 0)
        {
            cleste2++;
            ServoGhearaDreapta.setPosition(0.39);//0.16
        }
        else if(GhearaDreapta.wasJustPressed() && cleste2 == 1)
        {
            cleste2--;
            ServoGhearaDreapta.setPosition(0.918);
        }
    }

    void Brat()
    {

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

    void Intake()
    {

    }
}
