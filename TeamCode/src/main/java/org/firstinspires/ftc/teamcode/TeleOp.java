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
            MiscareBaza();
            SliderExtend();
            SliderBaza();
            Roteste();
            OutakeRotire();
            //SlidePoz();
            ActiuneAuto();
            //Cleste();
            //Intake();
            //Outake();
            telemetry.update();
            telemetry.addData("Rotire", ServoRotire.getPosition());
            telemetry.addData("OutakeRotire", OutakeDreapta.getPosition());
            telemetry.addData("SLider", SliderS.getCurrentPosition());
            telemetry.addData("OutakeSlide", PozSlider[pozitieOutake]);
            telemetry.addData("poz",Poz);

            /**IntakeSus.readValue();
            IntakeJos.readValue();
            if(IntakeSus.wasJustPressed()){
                BazaDreapta.setPosition(BazaDreapta.getPosition() + 0.01);
                BazaStanga.setPosition(BazaStanga.getPosition() + 0.01);
            }
            if(IntakeJos.wasJustPressed()){
                BazaDreapta.setPosition(BazaDreapta.getPosition() - 0.01);
                BazaStanga.setPosition(BazaStanga.getPosition() - 0.01);
            }*/
        }
    }
}