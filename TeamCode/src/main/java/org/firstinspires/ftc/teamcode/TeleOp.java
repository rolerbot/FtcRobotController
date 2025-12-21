package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RobotFTC", group="Linear Opmode")
public class TeleOp extends GlobalScope
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx imaginar = null;

    public void runOpMode()
    {
        imaginar = hardwareMap.get(DcMotorEx.class, "MotorulImaginar");
        Timer.startTime();

        Initialise();
        schimbator = 1.4 - schimbator;

        waitForStart();

        MapControlerButtons();

        while (opModeIsActive())
        {
<<<<<<< HEAD
            Specimen2();
            MiscareBaza();
            SliderPoz2();
            SliderBaza();
            Roteste();
            OutakeRotire();
            Nivel();
            Nivele();
            ActiuneAuto2();
            BazaExt();
            ParkButton();
            Cleste();
            Specimen();
            Pozitionare();
            InchideIntake();
            ButonRotire();
            telemetry.update();
=======
            ///  0 parametri la nici o functie...............
            ///  de asemenea, in dreptul fiecarei functie vreau o scurta descriere, este extrem de utila sa dai hover cu mouse-ul si sa vezi ce face
            MiscareBaza(); /// Functie buna
            SliderPoz2(); /// not ok
            SliderBaza(); // ce cauta asemenea functii in globalscope si nu in TeleOp? sunt apelate O SINGURA DATA!
            Roteste(); /// numar magic in 3 linii de cod... de ce?
            OutakeRotire(); // functie ok, sa dispara numerele magice... e o tema recurenta
            Nivel(); /// extrem de grav sa apelezi de 14 ori timer.seconds pentru un amarat de switch case...
            Nivele(); // 1 use iarasi, functia asta nu are ce cauta in globalscope.
            ActiuneAuto2(); /// functia asta e MULT MULT prea lunga!
            BazaExt(); /// ext ce?
            ParkButton(); /// functie chatgpt cu variabile magice
            Cleste(); /// ajutor, de cand e java case insensitive?
            Slider2Poz(); /// cat decat ok, schimba numele functiei
            Specimen(); /// codul repetitiv trebuie sa aiba o functie proprie. Daca vad usages 50 si sunt scrise in acelasi stil ceva nu e ok!
            Pozitionare(); /// din lista asta de functii nu imi pot da seama daca ordinea apelurilor e gandita
            /// sau scrisa la misto in ordinea in care au fost implementate functiile
            telemetry.update(); /// asta da! mai mult debugging! telemetry is your best friend!
>>>>>>> 2f88348 (Adaugat comentarii stil feedback. Acest repo devine arhiva.)
            telemetry.addData("Pivot", PivotIntake.getPosition());
            telemetry.addData("rotire", ServoRotire.getPosition());
            telemetry.addData("secunde", Timer.seconds());
            telemetry.addData("SlideCount", CountSpecimen);
        }
    }
}