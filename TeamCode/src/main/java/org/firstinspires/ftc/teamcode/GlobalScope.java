package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    public DcMotorEx SliderS = null;//Stanga
    public DcMotorEx SliderD = null;
    public Servo ServoRotire = null;
    public Servo OutakeStanga = null;
    public Servo OutakeDreapta = null;
    public Servo BazaStanga = null;
    public Servo BazaDreapta = null;
    public Servo IntakeStanga = null;
    public Servo IntakeDreapta = null;
    public Servo ServoGhearaIntake = null; //Cleste Stanga
    public Servo ServoGhearaOutake = null; //Cleste Dreapta

    void LinkComponents() {
        MotorFS = hardwareMap.get(DcMotorEx.class, "MotorFS");
        MotorFD = hardwareMap.get(DcMotorEx.class, "MotorFD");
        MotorSS = hardwareMap.get(DcMotorEx.class, "MotorSS");
        MotorSD = hardwareMap.get(DcMotorEx.class, "MotorSD");
        SliderS = hardwareMap.get(DcMotorEx.class, "SliderS");
        SliderD = hardwareMap.get(DcMotorEx.class, "SliderD");
        ServoGhearaOutake = hardwareMap.get(Servo.class, "ServoGhearaOutake");
        ServoGhearaIntake = hardwareMap.get(Servo.class, "ServoGhearaIntake");
        ServoRotire = hardwareMap.get(Servo.class, "ServoRotire");
        IntakeDreapta = hardwareMap.get(Servo.class, "IntakeDreapta");
        IntakeStanga = hardwareMap.get(Servo.class, "IntakeStanga");
        BazaDreapta = hardwareMap.get(Servo.class, "BazaDreapta");
        BazaStanga = hardwareMap.get(Servo.class, "BazaStanga");
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
    }

    void InitComponente() {

        BazaDreapta.setPosition(0.04);
        BazaStanga.setPosition(0.08);
        IntakeStanga.setPosition(0.649);
        IntakeDreapta.setPosition(0.6505);
        OutakeStanga.setPosition(0.4685);
        OutakeDreapta.setPosition(0.5717);
        ServoGhearaIntake.setPosition(0);
        ServoGhearaOutake.setPosition(0.0056);
        ServoRotire.setPosition(0.5);

    }

    void Controler() {
        InitComponente();

        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);

        Viteza = new ButtonReader(ct1, GamepadKeys.Button.B);
        IntakeSus = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        IntakeJos = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
        RotireStanga = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        RotireDreapta = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        OutakeJos = new ButtonReader(ct2, GamepadKeys.Button.DPAD_DOWN);
        OutakeSus = new ButtonReader(ct2, GamepadKeys.Button.DPAD_UP);
        RotireSus = new ButtonReader(ct2, GamepadKeys.Button.B);
        RotireJos = new ButtonReader(ct2, GamepadKeys.Button.X);
        SLiderJos = new ButtonReader(ct2, GamepadKeys.Button.A);
        SliderSus = new ButtonReader(ct2, GamepadKeys.Button.Y);
        //SliderMin = new ButtonReader(ct2, GamepadKeys.Button.LEFT_STICK_BUTTON);
        //SliderMax = new ButtonReader(ct2, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        Auto = new ButtonReader(ct1, GamepadKeys.Button.Y);
        NoAuto = new ButtonReader(ct1, GamepadKeys.Button.A);
        GhearaOutake = new TriggerReader(ct2, GamepadKeys.Trigger.RIGHT_TRIGGER);

        sus = new ButtonReader(ct2, GamepadKeys.Button.DPAD_LEFT);
        jos = new ButtonReader(ct2, GamepadKeys.Button.DPAD_RIGHT);
    }

    /// TELEOP
    double drive, strafe, twist;
    double[] speeds = new double[4];
    double schimbator = 0.4;//Viteza
    int pozitieIntake = 2, pozitieOutake = 0, pozitieSlide = 0;
    int PozSlideExt[] = {0, 900, 2400};
    double PozIntakeSt[] = {0.088, 0.168, 0.649, 1};
    double PozIntakeDr[] = {0.0905, 0.1705, 0.6505, 1};
    double PozOutakeDreapta[] = {0.5717, 0.4461, 0.375, 0.335, 0.2656};
    double PozOutakeStanga[] = {0.4685, 0.3405, 0.3405, 0.3405, 0.2183};
    int cnt = 0, timecounter = 1, secondtimer = 1;
    double CLesteInchis = 0.02 , ClesteDeschis = 0.0056;
    GamepadEx ct1, ct2;
    ButtonReader Viteza;
    /// cautator de viteze
    ButtonReader RotireStanga, RotireDreapta, RotireSus, RotireJos;
    ButtonReader IntakeSus, IntakeJos;
    ButtonReader OutakeJos, OutakeSus;
    ButtonReader SliderSus, SLiderJos;
    ButtonReader SliderMin, SliderMax;
    ButtonReader Auto, NoAuto;
    TriggerReader GhearaOutake;

    ButtonReader sus, jos;

    void MiscareBaza() {
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

    void SliderExtend() {
        double Controler = 0.05;
        if (gamepad2.right_stick_y > Controler && SliderS.getCurrentPosition() > 0) {
            // Coboara && Slider.getCurrentPosition() > cnta
            SliderD.setPower(-1);
            SliderS.setPower(-1);
        } else if (gamepad2.right_stick_y < -Controler && SliderS.getCurrentPosition() < 2400) {
            //Urca && Slider.getCurrentPosition() < cnt
            SliderS.setPower(1);
            SliderD.setPower(1);
        } else if (gamepad2.left_stick_y > Controler) {
            SliderD.setPower(-1);
            SliderS.setPower(-1);
        } else if (gamepad2.left_stick_y < -Controler) {
            SliderS.setPower(1);
            SliderD.setPower(1);
        } else {
            SliderS.setPower(0);
            SliderD.setPower(0);
        }
    }

    void SliderPoz(){
        SliderSus.readValue();
        SLiderJos.readValue();
        if(SliderSus.wasJustPressed() && pozitieSlide < 2){
            pozitieSlide++;
            SliderS.setTargetPosition(PozSlideExt[pozitieSlide]);
            SliderD.setTargetPosition(PozSlideExt[pozitieSlide]);
            SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SliderS.setPower(1);
            SliderD.setPower(1);
        }
        if(SLiderJos.wasJustPressed() && pozitieSlide > 0){
            pozitieSlide--;
            SliderS.setTargetPosition(PozSlideExt[pozitieOutake]);
            SliderD.setTargetPosition(PozSlideExt[pozitieOutake]);
            SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    void SliderPoz2(){
        SliderSus.readValue();
        SLiderJos.readValue();

        if(SliderSus.wasJustPressed()){
            if(SliderS.getCurrentPosition() < 650){
                SliderS.setTargetPosition(PozSlideExt[1]);
                SliderD.setTargetPosition(PozSlideExt[1]);
                SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setPower(1);
                SliderD.setPower(1);
            }
            else{
                SliderS.setTargetPosition(PozSlideExt[2]);
                SliderD.setTargetPosition(PozSlideExt[2]);
                SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
///-----------Jos---------------------------
        if(SLiderJos.wasJustPressed()){
            if(SliderS.getCurrentPosition() > 2300){
                SliderS.setTargetPosition(PozSlideExt[1]);
                SliderD.setTargetPosition(PozSlideExt[1]);
                SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else{
                SliderS.setTargetPosition(PozSlideExt[0]);
                SliderD.setTargetPosition(PozSlideExt[0]);
                SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }

    void OutakeRotire() {
        RotireSus.readValue();
        RotireJos.readValue();
        if (RotireSus.wasJustPressed())
            OutakeDreapta.setPosition(OutakeDreapta.getPosition() - 0.0027);
        if (RotireJos.wasJustPressed())
            OutakeDreapta.setPosition(OutakeDreapta.getPosition() + 0.0027);
    }

    void SliderBaza() {
        double Controler = 0.005;
        if (gamepad1.right_stick_y > Controler && BazaDreapta.getPosition() < 0.32 ||
                gamepad1.right_stick_y < -Controler && BazaStanga.getPosition() > 0.07) {
            BazaDreapta.setPosition(BazaDreapta.getPosition() + 0.0013 * gamepad1.right_stick_y);
            BazaStanga.setPosition(BazaStanga.getPosition() + 0.0013 * gamepad1.right_stick_y); //0.0025
        }
    }

    void Roteste() {
        double PosInitial = ServoRotire.getPosition();
        if (gamepad1.right_stick_x > 0.005 || gamepad1.right_stick_x < -0.005)
            ServoRotire.setPosition(PosInitial + 0.0023 * gamepad1.right_stick_x);
    }

    void Cleste() {
        GhearaOutake.readValue();
        if(GhearaOutake.wasJustPressed()){
            if(ServoGhearaOutake.getPosition() == CLesteInchis)
                ServoGhearaOutake.setPosition(ClesteDeschis);
            else ServoGhearaOutake.setPosition(CLesteInchis);
        }

    }

    void BazaExt() {
        Auto.readValue();
        NoAuto.readValue();
        if (Auto.wasJustPressed())
            cnt = 0;
        if (NoAuto.wasJustPressed())
            cnt = 1;
    }

    void ActiuneAuto() {
        ElapsedTime timp = new ElapsedTime();
        OutakeSus.readValue();
        OutakeJos.readValue();

        IntakeSus.readValue();
        IntakeJos.readValue();

        if (OutakeJos.wasJustPressed() && pozitieOutake > 0) {
            pozitieOutake--;
            OutakeStanga.setPosition(PozOutakeStanga[pozitieOutake]);
            OutakeDreapta.setPosition(PozOutakeDreapta[pozitieOutake]);
            if (pozitieOutake < 2) ServoGhearaOutake.setPosition(CLesteInchis);/// 0.022
            else ServoGhearaOutake.setPosition(ClesteDeschis);
            pozitieIntake = 1;
            timecounter = 1;
            secondtimer = 1;
        }
        if (OutakeSus.wasJustPressed() && pozitieOutake < 4) {
            pozitieOutake++;
            if (pozitieOutake == 1 && secondtimer > 0) {
                secondtimer = 0;
                resetRuntime();
            }
            if (pozitieOutake < 4) ServoGhearaOutake.setPosition(CLesteInchis);
            else ServoGhearaOutake.setPosition(CLesteInchis);//Deschis
            if (pozitieOutake > 1) {
                OutakeStanga.setPosition(0.48);
                OutakeStanga.setPosition(PozOutakeStanga[pozitieOutake]);
                OutakeDreapta.setPosition(PozOutakeDreapta[pozitieOutake]);//0.3405, 0.3517
            }
        }

        if (IntakeSus.wasJustPressed() && pozitieIntake < 3) {
            pozitieIntake++;
            if (pozitieIntake > 1) {
                OutakeStanga.setPosition(PozOutakeStanga[1]);
                OutakeDreapta.setPosition(PozOutakeDreapta[1]);
            } else ServoGhearaIntake.setPosition(0);

            if (pozitieIntake == 1 && cnt == 0) {
                BazaDreapta.setPosition(0.32);
                BazaStanga.setPosition(0.36);
            }
            if (pozitieIntake == 3) {
                BazaDreapta.setPosition(0.04);
                BazaStanga.setPosition(0.08);
            }
        }
        if (IntakeJos.wasJustPressed() && pozitieIntake > 0) {
            pozitieIntake--;
            if (pozitieIntake == 0) {
                OutakeStanga.setPosition(PozOutakeStanga[1]);
                OutakeDreapta.setPosition(PozOutakeDreapta[1]);
                ServoGhearaIntake.setPosition(0.022);
            } else if (pozitieIntake == 2 || pozitieIntake == 1)
                ServoGhearaIntake.setPosition(0.022);
            else ServoGhearaIntake.setPosition(0);

            if (pozitieIntake == 1 && cnt == 0) {
                BazaDreapta.setPosition(0.32);
                BazaStanga.setPosition(0.36);
            }
            if (pozitieIntake == 3) {
                BazaDreapta.setPosition(0.04);
                BazaStanga.setPosition(0.08);
            }
        }
        IntakeStanga.setPosition(PozIntakeSt[pozitieIntake]);
        IntakeDreapta.setPosition(PozIntakeDr[pozitieIntake]);

        if (time > 1 && timecounter > 0 && pozitieOutake == 1) {
            timecounter = 0;
            OutakeStanga.setPosition(0.48);
            OutakeStanga.setPosition(PozOutakeStanga[pozitieOutake]);
            OutakeDreapta.setPosition(PozOutakeDreapta[pozitieOutake]);//0.3405, 0.3517
        }
    }

    void ActiuneAuto3() {
        OutakeSus.readValue();
        OutakeJos.readValue();

        IntakeSus.readValue();
        IntakeJos.readValue();

        if(OutakeJos.wasJustPressed() && pozitieOutake > 0) {
            pozitieOutake--;
            OutakeStanga.setPosition(PozOutakeStanga[pozitieOutake]);
            OutakeDreapta.setPosition(PozOutakeDreapta[pozitieOutake]);
            if (pozitieOutake < 2) ServoGhearaOutake.setPosition(CLesteInchis);/// 0.022
            else ServoGhearaOutake.setPosition(ClesteDeschis);
            pozitieIntake = 1;
        }
        if (OutakeSus.wasJustPressed() && pozitieOutake < 4) {
            pozitieOutake++;
            OutakeStanga.setPosition(0.48);
            if (pozitieOutake < 4) ServoGhearaOutake.setPosition(ClesteDeschis);
            else ServoGhearaOutake.setPosition(CLesteInchis);//Deschis
            OutakeStanga.setPosition(PozOutakeStanga[pozitieOutake]);
            OutakeDreapta.setPosition(PozOutakeDreapta[pozitieOutake]);//0.3405, 0.3517
        }
        if (IntakeSus.wasJustPressed() && pozitieIntake < 3) {
            pozitieIntake++;
            if (pozitieIntake > 1) {
                OutakeStanga.setPosition(PozOutakeStanga[1]);
                OutakeDreapta.setPosition(PozOutakeDreapta[1]);
            } else ServoGhearaIntake.setPosition(0);
            if (pozitieIntake == 1 && cnt == 0) {
                BazaDreapta.setPosition(0.32);
                BazaStanga.setPosition(0.36);
            }
            if (pozitieIntake == 3) {
                BazaDreapta.setPosition(0.04);
                BazaStanga.setPosition(0.08);
            }
        }
        if (IntakeJos.wasJustPressed() && pozitieIntake > 0) {
            pozitieIntake--;
            if (pozitieIntake == 0) {
                OutakeStanga.setPosition(PozOutakeStanga[1]);
                OutakeDreapta.setPosition(PozOutakeDreapta[1]);
                ServoGhearaIntake.setPosition(0.022);
            } else if (pozitieIntake == 2 || pozitieIntake == 1) ServoGhearaIntake.setPosition(0.022);
            else ServoGhearaIntake.setPosition(0);
            if (pozitieIntake == 1 && cnt == 0) {
                BazaDreapta.setPosition(0.32);
                BazaStanga.setPosition(0.36);
            }
            if (pozitieIntake == 3) {
                BazaDreapta.setPosition(0.04);
                BazaStanga.setPosition(0.08);
            }
        }
        IntakeStanga.setPosition(PozIntakeSt[pozitieIntake]);
        IntakeDreapta.setPosition(PozIntakeDr[pozitieIntake]);
    }

    void ActiuneAuto2(){

        OutakeSus.readValue();
        OutakeJos.readValue();

        IntakeSus.readValue();
        IntakeJos.readValue();

        if(OutakeSus.wasJustPressed() && pozitieOutake < 4)
        {
            pozitieOutake++;
            if(pozitieIntake == 1)
            {

            }
            else if(pozitieIntake == 2)
            {

            }
            else if(pozitieIntake == 3)
            {

            }
            else
            {

            }
        }
        if(OutakeJos.wasJustPressed() && pozitieOutake > 0)
        {
          pozitieOutake--;
          if(pozitieOutake == 3)
          {

          }
          else if(pozitieOutake == 2)
          {

          }
          else if(pozitieOutake == 1)
          {

          }
          else
          {

          }
        }
        ///--------------------Intake-------------------------
        if(IntakeSus.wasJustPressed() && pozitieIntake < 3)
        {
            pozitieIntake++;
            if(pozitieIntake == 1)
            {

            }
            else if(pozitieIntake == 2)
            {

            }
            else
            {

            }

        }
        if(IntakeJos.wasJustPressed() && pozitieIntake > 0)
        {
            pozitieIntake--;
            if(pozitieOutake == 2)
            {

            }
            else if(pozitieOutake == 1)
            {

            }
            else
            {

            }
        }

        IntakeStanga.setPosition(PozIntakeSt[pozitieIntake]);
        IntakeDreapta.setPosition(PozIntakeDr[pozitieIntake]);
    }
}
