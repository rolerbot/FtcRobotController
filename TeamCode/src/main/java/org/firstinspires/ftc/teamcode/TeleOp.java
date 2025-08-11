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

            MiscareBaza();
            SliderPoz2();
            SliderBaza();
            Roteste();
            OutakeRotire();
            Nivel();
            Nivele();
            //ActiuneAuto();
            //ActiuneAuto3();
            ActiuneAuto2();
            BazaExt();
            ParkButton();
            Cleste();
            //Specimen();
            GasirePozitii(cLESTE1, cLESTE2, PivotIntake);
            telemetry.update();
            telemetry.addData("Secunde", Timer.seconds());
            telemetry.addData("Niv1", NivelNR);
        }
    }
}