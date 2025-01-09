package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RobotFTC", group="Linear Opmode")
public class TeleOp extends GlobalScope
{
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode()
    {
        Initialise();

        waitForStart();

        Controler();

        while (opModeIsActive())
        {
            SliderExtend();
            MiscareBaza();
            SliderPoz2();
            SliderBaza();
            Roteste();
            OutakeRotire();
            ActiuneAuto();
            BazaExt();
            ParkButton();
            Cleste();
            telemetry.update();
            telemetry.addData("OutakeSt", OutakeStanga.getPosition());
            telemetry.addData("OutakeDr", OutakeDreapta.getPosition());
            telemetry.addData("SliderSt", SliderS.getCurrentPosition());
            sus.readValue();
            jos.readValue();
            if(sus.wasJustPressed())
                OutakeStanga.setPosition(OutakeStanga.getPosition() + 0.007);
            if(jos.wasJustPressed())
                OutakeStanga.setPosition(OutakeStanga.getPosition() - 0.007);
        }
    }
}