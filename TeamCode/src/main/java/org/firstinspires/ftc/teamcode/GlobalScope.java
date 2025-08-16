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
    public DcMotorEx MotorIntake = null;
    public DcMotorEx SliderS = null;//Stanga
    public DcMotorEx SliderD = null;
    public Servo Parcare = null;
    public Servo ServoRotire = null;
    public Servo OutakeStanga = null;
    public Servo OutakeDreapta = null;
    public Servo BazaStanga = null;
    public Servo BazaDreapta = null;
    public Servo IntakeStanga = null;
    public Servo IntakeDreapta = null;
    public Servo ServoGhearaIntake = null; //Cleste Stanga
    public Servo ServoGhearaOutake = null;//Cleste Dreapta
    public  Servo PivotIntake = null;
    public ColorSensor colorSensor;


    void LinkComponents() {
        MotorFS = hardwareMap.get(DcMotorEx.class, "MotorFS");
        MotorFD = hardwareMap.get(DcMotorEx.class, "MotorFD");
        MotorSS = hardwareMap.get(DcMotorEx.class, "MotorSS");
        MotorSD = hardwareMap.get(DcMotorEx.class, "MotorSD");
        SliderS = hardwareMap.get(DcMotorEx.class, "SliderS");
        SliderD = hardwareMap.get(DcMotorEx.class, "SliderD");
        MotorIntake = hardwareMap.get(DcMotorEx.class, "MotorIntake");
        ServoGhearaOutake = hardwareMap.get(Servo.class, "ServoGhearaOutake");
        ServoGhearaIntake = hardwareMap.get(Servo.class, "ServoGhearaIntake");
        ServoRotire = hardwareMap.get(Servo.class, "ServoRotire");
        IntakeDreapta = hardwareMap.get(Servo.class, "IntakeDreapta");
        IntakeStanga = hardwareMap.get(Servo.class, "IntakeStanga");
        BazaDreapta = hardwareMap.get(Servo.class, "BazaDreapta");
        BazaStanga = hardwareMap.get(Servo.class, "BazaStanga");
        OutakeStanga = hardwareMap.get(Servo.class, "OutakeStanga");
        OutakeDreapta = hardwareMap.get(Servo.class, "OutakeDreapta");
        Parcare = hardwareMap.get(Servo.class, "Parcare");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        PivotIntake = hardwareMap.get(Servo.class, "PivotIntake");
    }

    void Initialise() {
        LinkComponents();
        //---------------------ROTZI---------------
        MotorFS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFS.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorSS.setDirection(DcMotorSimple.Direction.REVERSE);


        //--------------------------SLIDE-------------
        SliderS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SliderS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SliderS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SliderS.setDirection(DcMotorSimple.Direction.FORWARD);//Reverse

        SliderD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SliderD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SliderD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SliderD.setDirection(DcMotorSimple.Direction.FORWARD);//Reverse

        //------------------------SERVO---------------------
        ServoGhearaIntake.setDirection(Servo.Direction.FORWARD);
        ServoGhearaOutake.setDirection(Servo.Direction.FORWARD);
        ServoRotire.setDirection(Servo.Direction.FORWARD);
        BazaDreapta.setDirection(Servo.Direction.FORWARD);
        BazaStanga.setDirection(Servo.Direction.REVERSE);
        IntakeStanga.setDirection(Servo.Direction.FORWARD);
        IntakeDreapta.setDirection(Servo.Direction.REVERSE);
        OutakeStanga.setDirection(Servo.Direction.REVERSE);
        OutakeDreapta.setDirection(Servo.Direction.FORWARD);
        Parcare.setDirection(Servo.Direction.FORWARD);
        PivotIntake.setDirection(Servo.Direction.FORWARD);
    }

    void InitComponente() {

        BazaDreapta.setPosition(0.04);
        BazaStanga.setPosition(0.08);
        IntakeStanga.setPosition(0.737);//cv cu 0.6
        IntakeDreapta.setPosition(0.7372);
        OutakeStanga.setPosition(PozitiiOutake[0][0]);
        OutakeDreapta.setPosition(PozitiiOutake[1][0]);
        ServoGhearaIntake.setPosition(0);
        ServoGhearaOutake.setPosition(0.0185);//0.006
        ServoRotire.setPosition(0.4911);
        Parcare.setPosition(0.515);
        PivotIntake.setPosition(0.3);
    }

    void Controler() {
        InitComponente();

        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);

        Viteza = new ButtonReader(ct1, GamepadKeys.Button.B);
        IntakeSus = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        IntakeJos = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
        OutakeJos = new ButtonReader(ct2, GamepadKeys.Button.DPAD_DOWN);
        OutakeSus = new ButtonReader(ct2, GamepadKeys.Button.DPAD_UP);
        RotireSus = new ButtonReader(ct2, GamepadKeys.Button.DPAD_LEFT);
        RotireJos = new ButtonReader(ct2, GamepadKeys.Button.DPAD_RIGHT);
        SliderJos = new ButtonReader(ct2, GamepadKeys.Button.A);
        SliderSus = new ButtonReader(ct2, GamepadKeys.Button.Y);
        Park = new ButtonReader(ct2, GamepadKeys.Button.RIGHT_BUMPER);
        Auto = new ButtonReader(ct1, GamepadKeys.Button.Y);
        NoAuto = new ButtonReader(ct1, GamepadKeys.Button.A);
        GhearaOutakeDeschide = new TriggerReader(ct2, GamepadKeys.Trigger.RIGHT_TRIGGER);
        GhearaOutakeInchide = new ButtonReader(ct2, GamepadKeys.Button.LEFT_STICK_BUTTON);
        Specimen = new ButtonReader(ct2, GamepadKeys.Button.LEFT_BUMPER);
        NivelSlide1 = new ButtonReader(ct1, GamepadKeys.Button.LEFT_BUMPER);
        NivelSlide2 = new ButtonReader(ct1, GamepadKeys.Button.RIGHT_BUMPER);
        Specimen2 = new ButtonReader(ct2, GamepadKeys.Button.X);
        sus = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        jos = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        IntakePoz = new ButtonReader(ct1, GamepadKeys.Button.X);
        InchideInt = new ButtonReader(ct1, GamepadKeys.Button.LEFT_STICK_BUTTON);
        Rotire = new ButtonReader(ct1, GamepadKeys.Button.RIGHT_STICK_BUTTON);
    }

    /// TELEOP

    public ElapsedTime timpMiscare = new ElapsedTime();
    public ElapsedTime timpSlide = new ElapsedTime();
    double drive, strafe, twist;
    double[] speeds = new double[4];
    double schimbator = 0.4;//Viteza
    int pozitieIntake = 2, pozitieOutake = 0;
    int PozSlideExt[] = {0, 465, 905}; //0, 900, 2400
    double PozIntakeSt[] = {0.088, 0.2678, 0.73, 1}; //0.168
    double PozIntakeDr[] = {0.0905, 0.2711, 0.732, 1};//0.1705
    double PozOutakeDreapta[] = {0.5717, 0.4461, 0.3628, 0.335, 0.3078};//0.3361
    double PozOutakeStanga[] = {0.4685, 0.3405, 0.3405, 0.3405, 0.2872};
    double PozitiiIntake[][] ={ {0.088, 0.2106, 0.737}, {0.0905, 0.21, 0.7372} };
    double PozitiiOutake[][] ={ {0.385, 0.37, 0.3405, 0.2872},{0.508, 0.5078, 0.3628, 0.3078} };

    double Pivot[] = {0.205, 0.2089, 0.0483 , 0.0544};//0.0544, 0,0461
    int cnt = 0;
    int NivelNR = 0, PozTimer = 0;
    double CLesteInchis = 0.0056 , ClesteDeschis = 0.028;
    int Numarator = 0, nr = 0;
    int countSlide, CountSpecimen;
    public ElapsedTime Timer = new ElapsedTime();
    GamepadEx ct1, ct2;
    ButtonReader IntakePoz;
    ButtonReader NivelSlide1, NivelSlide2, Specimen2;
    ButtonReader Viteza;
    /// cautator de viteze
    ButtonReader RotireStanga, RotireDreapta, RotireSus, RotireJos;
    ButtonReader IntakeSus, IntakeJos;
    ButtonReader OutakeJos, OutakeSus;
    ButtonReader SliderSus, SliderJos, Park;
    ButtonReader Auto, NoAuto, Specimen, GhearaOutakeInchide;
    ButtonReader sus, jos;
    ButtonReader InchideInt, Rotire;
    TriggerReader GhearaOutakeDeschide;

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

    void InchideIntake()
    {
        InchideInt.readValue();
        if(InchideInt.wasJustPressed())
            ServoGhearaIntake.setPosition(ClesteDeschis);
    }

    void Pozitionare()
    {
        IntakePoz.readValue();
        if(IntakePoz.wasJustPressed())
        {
            if(PivotIntake.getPosition() <= 0.111 || PivotIntake.getPosition() >= 0.112)
            {
                PozTimer = 1;
                Timer.reset();
            }
            if(PivotIntake.getPosition() > 0.111 && PivotIntake.getPosition() < 0.112)
            {
                ServoGhearaIntake.setPosition(0.0235);
                BazaDreapta.setPosition(0.3);
                BazaStanga.setPosition(0.34);
                pozitieIntake = 1;
                PivotIntake.setPosition(Pivot[pozitieIntake]);
            }

        }

        if(PozTimer == 1)
        {
            ServoGhearaIntake.setPosition(CLesteInchis);
            if(Timer.seconds() > 0.2)
            {
                PivotIntake.setPosition(0.1117);
                IntakeStanga.setPosition(PozitiiIntake[0][1]);
                IntakeDreapta.setPosition(PozitiiIntake[1][1]);
                BazaDreapta.setPosition(0.04);
                BazaStanga.setPosition(0.08);
                PozTimer = 0;
            }
        }

    }

    void Specimen2()
    {
        Specimen2.readValue();
        if(Specimen2.wasJustPressed() && SliderS.getCurrentPosition() < 100)
        {
            Timer.reset();
            CountSpecimen = 1;
        }
        if(CountSpecimen == 1)
        {
            if(Timer.seconds() > 0 && Timer.seconds() < 0.2) ServoGhearaIntake.setPosition(CLesteInchis);
            if(Timer.seconds() > 0.2 && Timer.seconds() < 1.7)
            {
                pozitieIntake = 2;
                pozitieOutake = 1;
                ServoGhearaOutake.setPosition(ClesteDeschis);
                BazaDreapta.setPosition(0.04);
                BazaStanga.setPosition(0.08);
                IntakeStanga.setPosition(PozitiiIntake[0][pozitieIntake]);
                IntakeDreapta.setPosition(PozitiiIntake[1][pozitieIntake]);
                ServoRotire.setPosition(0.3772);
                PivotIntake.setPosition(Pivot[pozitieIntake]);
                OutakeStanga.setPosition(PozitiiOutake[0][pozitieOutake]);
                OutakeDreapta.setPosition(PozitiiOutake[1][pozitieOutake]);
            }
            if(Timer.seconds() > 1.7 && Timer.seconds() < 2.1)
            {
                pozitieOutake = 0;
                OutakeStanga.setPosition(PozitiiOutake[0][pozitieOutake]);
                OutakeDreapta.setPosition(PozitiiOutake[1][pozitieOutake]);
            }
            if(Timer.seconds() > 2.1 && Timer.seconds() < 2.3)  ServoGhearaOutake.setPosition(CLesteInchis);
            if(Timer.seconds() > 2.3 && Timer.seconds() < 2.5) ServoGhearaIntake.setPosition(ClesteDeschis);
            if(Timer.seconds() > 2.5 && Timer.seconds() < 3.2)
            {
                SliderS.setTargetPosition(251);
                SliderD.setTargetPosition(251);
                SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setPower(1);
                SliderD.setPower(1);
                pozitieOutake = 2;
                OutakeDreapta.setPosition(PozitiiOutake[1][pozitieOutake]);
                OutakeStanga.setPosition(PozitiiOutake[0][pozitieOutake]);
            }
            if(Timer.seconds() > 3.2) CountSpecimen = 0;
        }

        if(Specimen2.wasJustPressed() && SliderS.getCurrentPosition() > 100)
        {
            Timer.reset();
            Timer.startTime();
            SliderS.setTargetPosition(485);
            SliderD.setTargetPosition(485);
            SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SliderS.setPower(1);
            SliderD.setPower(1);
            CountSpecimen = 2;
        }
        if(CountSpecimen == 2 && Timer.seconds() > 0.7)
        {
           ServoGhearaOutake.setPosition(ClesteDeschis);
           CountSpecimen = 0;
        }
    }

    /**void Specimen()
    {
        Specimen.readValue();
        if(Specimen.wasJustPressed() && SliderS.getCurrentPosition() < 5)
        {
            timpSlide.reset();
            timpSlide.startTime();
            countSlide1 = 1;
            ServoGhearaOutake.setPosition(CLesteInchis);
        }
        else if(Specimen.wasJustPressed() && SliderS.getCurrentPosition() < 100)
        {
            countSlide2 = 1;
            timpSlide.reset();
            timpSlide.startTime();
            SliderS.setTargetPosition(251);
            SliderD.setTargetPosition(251);
            pozitieOutake = 2;
            SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            OutakeStanga.setPosition(PozOutakeStanga[pozitieOutake]);
            OutakeDreapta.setPosition(PozOutakeDreapta[pozitieOutake]);
        }
        if(countSlide1 == 1 && timpSlide.seconds() > 0.25)
        {
            SliderS.setTargetPosition(262);
            SliderD.setTargetPosition(262);
            SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SliderS.setPower(1);
            SliderD.setPower(1);
            pozitieOutake = 2;
            OutakeStanga.setPosition(PozOutakeStanga[pozitieOutake]);
            OutakeDreapta.setPosition(PozOutakeDreapta[pozitieOutake]);
            countSlide1 = 0;
        }
        if(countSlide2 == 1 && timpSlide.seconds() > 0.5){
            countSlide2 = 0;
            ServoGhearaOutake.setPosition(ClesteDeschis);
        }
    }*/

    /**
    void Specimen2()
    {
        Specimen2.readValue();
        if(Specimen2.wasJustPressed() && SliderS.getCurrentPosition() < 10)
        {
            pozitieOutake = 2;
            OutakeStanga.setPosition(PozOutakeStanga[3]);
            OutakeDreapta.setPosition(PozOutakeDreapta[3]);
            SliderS.setTargetPosition(330);
            SliderD.setTargetPosition(330);
            SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SliderD.setPower(1);
            SliderS.setPower(1);
        }
        if(Specimen2.wasJustPressed() && SliderS.getCurrentPosition() > 10)
        {
            timpSlide.reset();
            timpSlide.startTime();
            countSlide2 = 1;
        }
        if(countSlide2 == 1)
        {
            if(timpSlide.seconds() > 0 && timpSlide.seconds() < 2)
            {
                SliderS.setTargetPosition(170);
                SliderD.setTargetPosition(170);
                SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setPower(1);
                SliderD.setPower(1);
            }
            if(timpSlide.seconds() > 2)
            {
                ServoGhearaOutake.setPosition(ClesteDeschis);
                countSlide2 = 0;
            }
        }

    }*/

    void Specimen()
    {
        Specimen.readValue();
        if(Specimen.wasJustPressed() && SliderS.getCurrentPosition() < 10)
        {
            timpSlide.reset();
            timpSlide.startTime();
            countSlide = 1;
            ServoGhearaOutake.setPosition(CLesteInchis);
        }
        if(countSlide == 1)
        {
            if(timpSlide.seconds() > 0.2 && timpSlide.seconds() < 0.5)
            {
                SliderS.setTargetPosition(251);
                SliderD.setTargetPosition(251);
                SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setPower(1);
                SliderD.setPower(1);
                pozitieOutake = 2;
                OutakeDreapta.setPosition(PozitiiOutake[1][pozitieOutake]);
                OutakeStanga.setPosition(PozitiiOutake[0][pozitieOutake]);
            }
            if(timpSlide.seconds() > 0.5) countSlide = 0;
        }

        if(Specimen.wasJustPressed() && SliderS.getCurrentPosition() > 10)
        {
            timpSlide.reset();
            timpSlide.startTime();
            countSlide = 2;
        }
        if(countSlide == 2)
        {
            if(timpSlide.seconds() < 0.8)
            {
                SliderS.setTargetPosition(480);
                SliderD.setTargetPosition(480);
                SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setPower(1);
                SliderD.setPower(1);
            }
            if(timpSlide.seconds() > 0.8 && timpSlide.seconds() < 1)
                ServoGhearaOutake.setPosition(ClesteDeschis);
            if(timpSlide.seconds() > 1) countSlide = 0;
        }
    }

    void SliderPoz2()
    {
        SliderSus.readValue();
        SliderJos.readValue();

        if(SliderSus.wasJustPressed())
        {
            if(SliderS.getCurrentPosition() < 300)
            {
                SliderS.setTargetPosition(PozSlideExt[1]);
                SliderD.setTargetPosition(PozSlideExt[1]);
                SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setPower(1);
                SliderD.setPower(1);
            }
            else
            {
                SliderS.setTargetPosition(PozSlideExt[2]);
                SliderD.setTargetPosition(PozSlideExt[2]);
                SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setPower(1);
                SliderD.setPower(1);
            }
        }
///-----------Jos---------------------------
        if(SliderJos.wasJustPressed())
        {
            if(SliderS.getCurrentPosition() > 800)
            {
                SliderS.setTargetPosition(PozSlideExt[1]);
                SliderD.setTargetPosition(PozSlideExt[1]);
                SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setPower(0.7);
                SliderD.setPower(0.7);
            }
            else
            {
                SliderS.setTargetPosition(PozSlideExt[0]);
                SliderD.setTargetPosition(PozSlideExt[0]);
                SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderD.setPower(0.7);
                SliderS.setPower(0.7);
            }
        }
    }

    void ParkButton()
    {
        Park.readValue();
        if(Park.wasJustPressed()){
            if(Parcare.getPosition() == 0.515)
                Parcare.setPosition(0.6172);
            else Parcare.setPosition(0.515);
        }
    }

    void OutakeRotire()
    {
        RotireSus.readValue();
        RotireJos.readValue();
        if (RotireSus.wasJustPressed())
            OutakeDreapta.setPosition(OutakeDreapta.getPosition() - 0.003);
        if (RotireJos.wasJustPressed())
            OutakeDreapta.setPosition(OutakeDreapta.getPosition() + 0.003);
    }

    void SliderBaza()
    {
        double Controler = 0.005;
        if (gamepad1.right_stick_y > Controler && BazaDreapta.getPosition() < 0.28 ||
                gamepad1.right_stick_y < -Controler && BazaStanga.getPosition() > 0.07) {
            BazaDreapta.setPosition(BazaDreapta.getPosition() + 0.0013 * gamepad1.right_stick_y);
            BazaStanga.setPosition(BazaStanga.getPosition() + 0.0013 * gamepad1.right_stick_y); //0.0025
        }// < 0.32
    }

    void ButonRotire()
    {
        Rotire.readValue();
        if(Rotire.wasJustPressed()) ServoRotire.setPosition(0.4911);
    }

    void Roteste()
    {
        double PosInitial = ServoRotire.getPosition();
        if (gamepad1.right_stick_x > 0.005 && ServoRotire.getPosition() < 0.56)
            ServoRotire.setPosition(PosInitial + 0.00071 * gamepad1.right_stick_x);
        if (gamepad1.right_stick_x < -0.005 && ServoRotire.getPosition() > 0.38)
            ServoRotire.setPosition(PosInitial + 0.00071 * gamepad1.right_stick_x);
    }

    void Cleste()
    {
        GhearaOutakeDeschide.readValue();
        GhearaOutakeInchide.readValue();
        if(GhearaOutakeDeschide.wasJustPressed())
            ServoGhearaOutake.setPosition(ClesteDeschis);
        if(GhearaOutakeInchide.wasJustPressed())
            ServoGhearaOutake.setPosition(CLesteInchis);
    }

    void BazaExt()
    {
        Auto.readValue();
        NoAuto.readValue();
        if (Auto.wasJustPressed())
            cnt = 0;
        if (NoAuto.wasJustPressed())
            cnt = 1;
    }

   void ActiuneAuto2()
   {
       OutakeSus.readValue();
       OutakeJos.readValue();
       IntakeSus.readValue();
       IntakeJos.readValue();

       if(IntakeJos.wasJustPressed() && pozitieIntake > 0 ) //poz 0
       {
           if(pozitieIntake > 1 && cnt == 0)
           {
               BazaDreapta.setPosition(0.3);
               BazaStanga.setPosition(0.34);
           }
           pozitieIntake--;
           if(pozitieOutake == 0) ServoGhearaOutake.setPosition(ClesteDeschis);
           IntakeStanga.setPosition(PozitiiIntake[0][pozitieIntake]);
           IntakeDreapta.setPosition(PozitiiIntake[1][pozitieIntake]);
           PivotIntake.setPosition(Pivot[pozitieIntake]);
           if(pozitieIntake > 1) ServoGhearaIntake.setPosition(CLesteInchis);
           else ServoGhearaIntake.setPosition(0.0235);
       }

       if(IntakeSus.wasJustPressed() && pozitieIntake == 0)
       {
           nr = 1;
           ServoGhearaIntake.setPosition(CLesteInchis);
           Timer.reset();
           pozitieIntake++;
       }

       if(nr == 1)
       {
           if(Timer.seconds() > 0.2 && Timer.seconds() < 0.5)
           {
               ServoRotire.setPosition(0.4911);
               IntakeStanga.setPosition(PozitiiIntake[0][pozitieIntake]);
               IntakeDreapta.setPosition(PozitiiIntake[1][pozitieIntake]);
               PivotIntake.setPosition(Pivot[pozitieIntake]);
           }
           if(Timer.seconds() > 0.5 && Timer.seconds() < 1)
           {
               BazaDreapta.setPosition(0.04);
               BazaStanga.setPosition(0.08);
           }
           if(Timer.seconds() > 1) nr = 0;
       }

       if(IntakeSus.wasJustPressed() && pozitieIntake == 1) //poz 1
       {
           Numarator = 1;
           Timer.reset();
           pozitieIntake++;
       }

       if(Numarator == 1) //poz 3
       {
           pozitieOutake = 1;
           if (Timer.seconds() > 0.1 && Timer.seconds() < 0.7)
           {
               OutakeStanga.setPosition(PozitiiOutake[0][1]);
               OutakeDreapta.setPosition(PozitiiOutake[1][1]);
           }
           if(Timer.seconds() > 0.7 && Timer.seconds() < 1.5)
           {
               IntakeStanga.setPosition(PozitiiIntake[0][pozitieIntake]);
               IntakeDreapta.setPosition(PozitiiIntake[1][pozitieIntake]);
               ServoGhearaOutake.setPosition(ClesteDeschis);
               PivotIntake.setPosition(Pivot[pozitieIntake]);
           }
           if(Timer.seconds() > 1.7 && Timer.seconds() < 2)
           {
               OutakeDreapta.setPosition(PozitiiOutake[1][0]);
               OutakeStanga.setPosition(PozitiiOutake[0][0]);
           }
           if (Timer.seconds() > 2 && Timer.seconds() < 2.15)
               ServoGhearaOutake.setPosition(CLesteInchis);
           if (Timer.seconds() > 2.2 && Timer.seconds() < 2.4)
               ServoGhearaIntake.setPosition(0.0235);
           if(Timer.seconds() > 2.4 && Timer.seconds() < 2.7)
           {
               OutakeDreapta.setPosition(PozitiiOutake[1][2]);
               OutakeStanga.setPosition(PozitiiOutake[0][2]);
           }
           if (Timer.seconds() > 2.7)
              {
               Numarator = 0;
               pozitieOutake = 2;
           }
       }

       if(OutakeSus.wasJustPressed() && pozitieOutake < 3)
       {
           pozitieOutake++;
           OutakeStanga.setPosition(PozitiiOutake[0][pozitieOutake]);
           OutakeDreapta.setPosition(PozitiiOutake[1][pozitieOutake]);
       }
       if(OutakeJos.wasJustPressed() && pozitieOutake > 0)
       {
           pozitieOutake--;
           OutakeStanga.setPosition(PozitiiOutake[0][pozitieOutake]);
           OutakeDreapta.setPosition(PozitiiOutake[1][pozitieOutake]);
           if(pozitieOutake == 0) ServoGhearaOutake.setPosition(ClesteDeschis);
       }
    }

    void Nivele()
    {
        NivelSlide1.readValue();
        NivelSlide2.readValue();
        if(NivelSlide1.wasJustPressed())
        {
            NivelNR = 1;
            Timer.reset();
        }
        if(NivelSlide2.wasJustPressed())
        {
            NivelNR = 2;
            Timer.reset();
        }
    }

    void Nivel()
    {
        if(NivelNR != 0)
        {
            if(Timer.seconds() > 0 && Timer.seconds() < 0.2)
            {
                pozitieIntake = 2;
                ServoGhearaIntake.setPosition(CLesteInchis);
            }
            if(Timer.seconds() > 0.2 && Timer.seconds() < 1.5)
            {
                ServoRotire.setPosition(0.4911);
                BazaDreapta.setPosition(0.04);
                BazaStanga.setPosition(0.08);
                ServoGhearaOutake.setPosition(ClesteDeschis);
                IntakeStanga.setPosition(PozitiiIntake[0][2]);
                IntakeDreapta.setPosition(PozitiiIntake[1][2]);
                PivotIntake.setPosition(Pivot[2]);
                OutakeStanga.setPosition(PozitiiOutake[0][1]);
                OutakeDreapta.setPosition(PozitiiOutake[1][1]);
            }
            if(Timer.seconds() > 1.5 && Timer.seconds() < 1.7)
            {
                OutakeStanga.setPosition(PozitiiOutake[0][0]);
                OutakeDreapta.setPosition(PozitiiOutake[1][0]);
            }
            if(Timer.seconds() > 1.7 && Timer.seconds() < 1.9)
                ServoGhearaOutake.setPosition(CLesteInchis);
            if(Timer.seconds() > 1.9 && Timer.seconds() < 1.95)
                ServoGhearaIntake.setPosition(ClesteDeschis);
            if(Timer.seconds() > 1.95 &&  Timer.seconds() < 2.7)
            {
                pozitieOutake = 2;
                OutakeDreapta.setPosition(PozitiiOutake[1][pozitieOutake]);
                OutakeStanga.setPosition(PozitiiOutake[0][pozitieOutake]);
                SliderD.setTargetPosition(PozSlideExt[NivelNR]);
                SliderS.setTargetPosition(PozSlideExt[NivelNR]);
                SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setPower(1);
                SliderD.setPower(1);
            }
            if(Timer.seconds() > 2.7)
                NivelNR = 0;
        }
    }

}
