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

        waitForStart();

        Controler();

        while (opModeIsActive())
        {
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
            telemetry.addData("Pivot", PivotIntake.getPosition());
            telemetry.addData("rotire", ServoRotire.getPosition());
            telemetry.addData("secunde", Timer.seconds());
            telemetry.addData("SlideCount", CountSpecimen);
        }
    }
}