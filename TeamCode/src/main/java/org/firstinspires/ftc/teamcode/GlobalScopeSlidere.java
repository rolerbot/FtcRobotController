package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class GlobalScopeSlidere extends LinearOpMode
{
    public DcMotorEx MotorFS = null; /// Fata stanga
    public DcMotorEx MotorFD = null; /// Fata dreapta
    public DcMotorEx MotorSS = null; /// Spate stanga
    public DcMotorEx MotorSD = null; /// Spate dreapta
    public DcMotorEx MotorSlider = null;
    public DcMotorEx MotorIntake = null;
    public TouchSensor RevButon = null; //Buton oprire
    public Servo ServoIntake = null;
    public Servo ServoRotire = null;
    public Servo ServoStanga = null;// Servo Stanga
    public Servo ServoDreapta = null; //Servo Dreapta
    public Servo ServoDrona = null; // Servo Drona
    public Servo ServoGhearaDreapta = null; //Cleste Stanga
    public Servo ServoGhearaStanga = null; //Cleste Dreapta
    public Servo ServoBrat = null;
    public ColorRangeSensor SenzorStanga = null; //Senzor culoare Stang
    public ColorRangeSensor SenzorDreapta = null; //Senzor culoare drept

    void LinkComponents() {
        MotorFS = hardwareMap.get(DcMotorEx.class, "MotorFS");
        MotorFD = hardwareMap.get(DcMotorEx.class, "MotorFD");
        MotorSS = hardwareMap.get(DcMotorEx.class, "MotorSS");
        MotorSD = hardwareMap.get(DcMotorEx.class, "MotorSD");
        MotorSlider = hardwareMap.get(DcMotorEx.class, "SliderStanga");
        RevButon = hardwareMap.get(TouchSensor.class, "RevButon");
        ServoDreapta = hardwareMap.get(Servo.class, "SDreapta");
        ServoStanga = hardwareMap.get(Servo.class, "SStanga");
        ServoDrona = hardwareMap.get(Servo.class, "SDrona");
        ServoBrat = hardwareMap.get(Servo.class, "SBrat");
        SenzorStanga = hardwareMap.get(ColorRangeSensor.class, "Color1");
        SenzorDreapta = hardwareMap.get(ColorRangeSensor.class, "Color2");
        ServoGhearaStanga = hardwareMap.get(Servo.class, "CDreapta");
        ServoGhearaDreapta = hardwareMap.get(Servo.class, "CStanga");
        ServoRotire = hardwareMap.get(Servo.class, "SRotire");
        MotorIntake = hardwareMap.get(DcMotorEx.class, "MotorI");
        ServoIntake = hardwareMap.get(Servo.class, "Intake");
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
        //MotorFS.setTargetPositionTolerance(5); /// Eroare de 10 unitati I guess idk
        //MotorFD.setTargetPositionTolerance(5);
        //MotorSS.setTargetPositionTolerance(5);
        //MotorSD.setTargetPositionTolerance(5);
        MotorFS.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorSS.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //--------------------------BRATZ-------------
        MotorSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSlider.setDirection(DcMotorSimple.Direction.FORWARD);

        //------------------------Servo---------------------
        ServoBrat.setDirection(Servo.Direction.REVERSE);
        ServoStanga.setDirection(Servo.Direction.REVERSE);
        ServoDreapta.setDirection(Servo.Direction.FORWARD);
        ServoDrona.setDirection(Servo.Direction.FORWARD); //Vedem daca trebuie Reverse
        ServoGhearaDreapta.setDirection(Servo.Direction.FORWARD);
        ServoGhearaStanga.setDirection(Servo.Direction.REVERSE);
        ServoRotire.setDirection(Servo.Direction.FORWARD);
        ServoRotire.scaleRange(0,0.2);
        ServoIntake.setDirection(Servo.Direction.REVERSE);
    }

    /// TELEOP
    Gamepad.RumbleEffect customRumbleEffect;
    double drive;
    double strafe;
    double twist;
    double[] speeds = new double[4];
    double schimbator = 0.4;
    int cnt = 8000;
    int ok = 0;
    double vit = 1; //Viteza
    int c1 = 0, c2 = 0, b = 0, t = 0, pozitieActualaIntake = 0, n = 0, senzor = 0;
    double pozitiiIntake[] = {0.005, 0.275, 0.036};
    GamepadEx ct1, ct2;
    ButtonReader IAMSPEED; /// cautator de viteze
    ButtonReader Vit, Launch;
    ButtonReader RotireStanga, RotireDreapta;
    ButtonReader BratSus, BratJos;
    ButtonReader VitezaPozitivaIntake, IntakeUp, IntakeDown;
    TriggerReader GhearaStanga, GhearaDreapta;
    ButtonReader VitezaNegativaIntake;

    void WeGottaMove()
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

    void WeGottaExtend()
    {
        if (gamepad2.left_stick_y > 0.5 && !RevButon.isPressed()) // Coboara
        {
            MotorSlider.setPower(-vit);
            ok = 0;
        }
        else if (gamepad2.left_stick_y < -0.5 && MotorSlider.getCurrentPosition() < cnt) //Urca
            MotorSlider.setPower(vit);
        else
            MotorSlider.setPower(0);
        if(RevButon.isPressed() && ok == 0)
        {
            MotorSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MotorSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ok = 1;
        }
    }

    void SlideVit()
    {
        Vit.readValue();
        if (Vit.wasJustPressed() && vit == 1)
            vit = 0.5;
        else if (Vit.wasJustPressed() && vit == 0.5)
            vit = 1;
    }

    void WeGottaLunchDrone()
    {
        Launch.readValue();
        if(Launch.wasJustPressed())
        {
            ServoDrona.setPosition(0.4);/// vedem daca 0.3 e bine, deocamdata e pus la misto
            //sleep(1500);
            //ServoDrona.setPosition(0.6);
        }
    }

    void SenzoriCuloare()
    {
        if (SenzorStanga.getDistance(DistanceUnit.MM) < 55 && SenzorDreapta.getDistance(DistanceUnit.MM) < 55 && senzor == 0)
        {
            senzor = 1;
            gamepad1.rumble(0,200,1000);
            gamepad1.rumble(0,200,1000);
            gamepad1.rumble(0,0,200);
            gamepad1.rumble(0,0,200);
            gamepad1.rumble(0,200,1000);
            gamepad1.rumble(0,200,1000);
        }
        else if (SenzorStanga.getDistance(DistanceUnit.MM) < 55 && senzor == 0)
        {
            gamepad1.rumble(0,200,1000);
            gamepad1.rumble(0,200,1000);
            senzor = 1;
        }
        else if(SenzorDreapta.getDistance(DistanceUnit.MM) < 55 && senzor == 0)
        {
            gamepad1.rumble(0,200,1000);
            gamepad1.rumble(0,200,1000);
            senzor = 1;
        }
        if(SenzorDreapta.getDistance(DistanceUnit.MM) > 55 && SenzorStanga.getDistance(DistanceUnit.MM) > 55)
            senzor = 0;
    }

    void SenzorSiCleste()
    {
        GhearaStanga.readValue();
        if(ServoBrat.getPosition()<0.2 && ServoStanga.getPosition()<0.2) {
            if (SenzorStanga.getDistance(DistanceUnit.MM) < 58) {
                //sleep(1000);
                //ServoGhearaDreapta.setPosition(0.06);
                gamepad1.rumble(0,200,1000);
            } else {
                //ServoGhearaDreapta.setPosition(0.64);
            }
            if (SenzorDreapta.getDistance(DistanceUnit.MM) < 58) {
                //sleep(1000);
                //ServoGhearaStanga.setPosition(0.06);
                gamepad1.rumble(200,0,1000);
            } else {
                //ServoGhearaStanga.setPosition(0.64);

            }
        }
    }

    void Cleste()
    {
        GhearaStanga.readValue();
        GhearaDreapta.readValue();

        if(GhearaStanga.wasJustPressed() && c1 == 0)
        {
            c1++;
            ServoGhearaStanga.setPosition(0.6);
        }
        else if(GhearaStanga.wasJustPressed() && c1 == 1)
        {
            c1--;
            ServoGhearaStanga.setPosition(0);
        }

        if(GhearaDreapta.wasJustPressed() && c2 == 0)
        {
            c2++;
            ServoGhearaDreapta.setPosition(0.918);//0.16
        }
        else if(GhearaDreapta.wasJustPressed() && c2 == 1)
        {
            c2--;
            ServoGhearaDreapta.setPosition(0.39);//0.6// cu cleste 1
        }
    }

    void Brat()
    {
        BratSus.readValue();
        if(BratSus.wasJustPressed() && b == 0)
        {
            ServoStanga.setPosition(0.76);
            ServoDreapta.setPosition(0.59);
            sleep(100);
            ServoBrat.setPosition(0.05);
            sleep(400);
            ServoBrat.setPosition(0.63);
            b++;
        }
        BratJos.readValue();
        if(BratJos.wasJustPressed() && b == 1)
        {
            ServoRotire.setPosition(0.5);
            ServoGhearaStanga.setPosition(0);
            ServoGhearaDreapta.setPosition(0.3875);
            ServoBrat.setPosition(0.153);
            sleep(150);
            ServoStanga.setPosition(0.187);//0.19
            ServoDreapta.setPosition(0.017); //0.02
            sleep(700);
            ServoBrat.setPosition(0.171);//0.19/0.173
            b--;
        }
    }

    void Roteste()
    {
        RotireStanga.readValue();
        RotireDreapta.readValue();
        double posRotire = ServoRotire.getPosition();
        if(RotireStanga.wasJustPressed())
            ServoRotire.setPosition(posRotire - 0.125);
        if(RotireDreapta.wasJustPressed())
            ServoRotire.setPosition(posRotire +0.125);
    }

    void Intake()
    {
        VitezaPozitivaIntake.readValue();
        if(VitezaPozitivaIntake.wasJustPressed() && t == 0){
            MotorIntake.setPower(0.6);
            t++;
        }
        else if(VitezaPozitivaIntake.wasJustPressed() && t == 1)
        {
            MotorIntake.setPower(0);
            t--;
        }
        VitezaNegativaIntake.readValue();
        if(VitezaNegativaIntake.wasJustPressed() && n == 0)
        {
            MotorIntake.setPower(-0.6);
            n++;
        }
        else if(VitezaNegativaIntake.wasJustPressed() && n == 1)
        {
            MotorIntake.setPower(0);
            n--;
        }

        IntakeDown.readValue();
        IntakeUp.readValue();

        if(IntakeDown.wasJustPressed() && pozitieActualaIntake >= 1)
        {
            pozitieActualaIntake--;
            ServoIntake.setPosition(pozitiiIntake[pozitieActualaIntake]);
        }
        if(IntakeUp.wasJustPressed() && pozitieActualaIntake <= 1)
        {
            pozitieActualaIntake++;
            ServoIntake.setPosition(pozitiiIntake[pozitieActualaIntake]);
        }
    }
}
