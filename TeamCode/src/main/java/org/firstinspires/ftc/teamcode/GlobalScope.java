package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public abstract class GlobalScope extends LinearOpMode
{
    public DcMotorEx MotorFS = null;
    /// Fata stanga
    public DcMotorEx MotorFD = null;
    /// Fata dreapta
    public DcMotorEx MotorSS = null;
    /// Spate stanga
    public DcMotorEx MotorSD = null;
    /// Spate dreapta
    public DcMotorEx MotorIN = null;
    /// Servouri Mixer
    public Servo ServoMixer1 = null;
    public Servo ServoMixer2 = null;
    ///  Servo pentru aruncare
    public Servo ServoRidicare = null;
    /// Motor Aruncare
    public  DcMotorEx MotorAruncare = null;
    public void LinkComponents()
    {
        MotorFS = hardwareMap.get(DcMotorEx.class, "MotorFS");
        MotorFD = hardwareMap.get(DcMotorEx.class, "MotorFD");
        MotorSS = hardwareMap.get(DcMotorEx.class, "MotorSS");
        MotorSD = hardwareMap.get(DcMotorEx.class, "MotorSD");
        MotorIN = hardwareMap.get(DcMotorEx.class, "MotorIN");
        ServoMixer1 = hardwareMap.get(Servo.class, "ServoMixer1");
        ServoMixer2 = hardwareMap.get(Servo.class, "ServoMixer2");
        ServoRidicare = hardwareMap.get(Servo.class, "ServoRidicare");
        MotorAruncare = hardwareMap.get(DcMotorEx.class, "MotorAruncare");
    }

    void Initialise()
    {
        LinkComponents();

        //---------------------ROTZI----------------

        MotorFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFD.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorFS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFS.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorSD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSD.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorSS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSS.setDirection(DcMotorSimple.Direction.FORWARD);

        //---------------------ARUNCARE SI INTAKE----------------
        MotorAruncare.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorAruncare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare.setDirection(DcMotorSimple.Direction.FORWARD);

        MotorIN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorIN.setDirection(DcMotorSimple.Direction.FORWARD);

        //------------------------SERVO---------------------

        ServoMixer1.setDirection(Servo.Direction.FORWARD);
        ServoMixer2.setDirection(Servo.Direction.FORWARD);
        ServoRidicare.setDirection(Servo.Direction.FORWARD);
    }

    void MapControlerButtons()
    {
        InitComponente();

        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);

        Viteza = new ButtonReader(ct1, GamepadKeys.Button.B);
        ButtonSus = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        ButtonJos = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
        Aruncare = new ButtonReader(ct1, GamepadKeys.Button.A);
    }

    void InitComponente()
    {
        ServoMixer1.setPosition(0);
        ServoMixer2.setPosition(0);
        ServoRidicare.setPosition(0);
    }

    /// TELEOP

    double drive, strafe, twist;
    int counterRotire = 0;
    int counterInversare = 0;
    double[] speeds = new double[4];
    double[] pozitiiIndx = {0, 0.2, 0.4}; //3 pozitii
    double[] pozitiiAruncare = {0.1, 0.3, 0.5}; // 3 pozitii aruncare
    enum culoare {mov, verde};
    private ElapsedTime runtime = new ElapsedTime();
    int lenPozitii = 0;
    culoare[] artifacte = new culoare[3];
    double schimbator = 0.4;//Viteza 0.4
    GamepadEx ct1, ct2;
    ButtonReader Viteza;
    ColorSensor cSensorSt, cSensorDr;
    ButtonReader ButtonSus, ButtonJos, Aruncare;

    void MiscareBaza()
    {
        Viteza.readValue();
        if (Viteza.wasJustPressed())
        {
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
        for (int i = 0; i < speeds.length; i++)
        {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }
        if (max > 1)
        {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
        MotorFS.setPower(speeds[0]);
        MotorFD.setPower(speeds[1]);
        MotorSS.setPower(speeds[2]);
        MotorSD.setPower(speeds[3]);
    }

    void MotorIntake()
    {
        ButtonSus.readValue();

        if (ButtonSus.wasJustPressed() && counterRotire == 1)
        {
            counterRotire = 0;
            MotorIN.setPower(0);
        } else if (ButtonSus.wasJustPressed() && counterRotire == 0)
        {
            counterRotire = 1;
            MotorIN.setPower(0.8);
        }
    }

    void MotorIntakeReverse()
    {
        ButtonJos.readValue();

        if (ButtonJos.wasJustPressed() && counterInversare == 1)
        {
            counterRotire = 0;
            counterInversare = 0;
            MotorIN.setPower(0);
        } else if (ButtonJos.wasJustPressed() && counterInversare == 0)
        {
            counterRotire = 0;
            counterInversare = 1;
            MotorIN.setPower(-0.8);
        }
    }

    private int Culoare(ColorSensor cSensor)
    {
        int red = cSensor.red();
        int green = cSensor.green();
        int blue = cSensor.blue();

        if (red > green && blue > green && red > 100 && blue > 100)
            return 1; // Mov

        if (green > red && green > blue && green > 100)
            return 2; // Verde

        return 0; // Altă culoare
    }

    void ArtifacteIndx()
    {
        if (runtime.seconds() == 0 && (Culoare(cSensorSt) > 0 || Culoare(cSensorDr) > 0) && counterRotire == 1)
        {
            runtime.startTime();
            if (Culoare(cSensorSt) == 1 || Culoare(cSensorDr) == 1)
                artifacte[lenPozitii++] = culoare.mov;
            else if (Culoare(cSensorSt) == 2 || Culoare(cSensorDr) == 2)
                artifacte[lenPozitii++] = culoare.verde;
        }
        else if (runtime.seconds() > 1)
        {
            if (lenPozitii == 3)
            {
                counterRotire = 0;
                MotorIN.setPower(0);
            }
            else
            {
                ServoMixer1.setPosition(pozitiiIndx[lenPozitii]);
                //ServoMixer2.setPosition(pozitiiIndx[lenPozitii]);
            }
            runtime.reset();
        }
    }

    void AruncareArtifacte()
    {
        Aruncare.readValue();
        if (counterRotire == 0 && lenPozitii > 0 && Aruncare.wasJustPressed() && runtime.seconds() == 0)
        {
            runtime.reset();
            runtime.startTime();
            ServoMixer1.setPosition(pozitiiAruncare[--lenPozitii]);

            //ServoMixer2.setPosition(pozitiiAruncare[--lenPozitii]);
        }
    }

    void GasirePozitii(ButtonReader x, ButtonReader y, Servo Test, Servo test)
    {
        x.readValue();
        y.readValue();
        double pozitie = Test.getPosition();
        double pozitie2 = test.getPosition();
        if (x.wasJustPressed())
        {
            test.setPosition(pozitie2 + 0.001);
            Test.setPosition(pozitie + 0.001);
        }
        if (y.wasJustPressed())
        {
            test.setPosition(pozitie - 0.001);
            Test.setPosition(pozitie - 0.001);
        }
    }
}
