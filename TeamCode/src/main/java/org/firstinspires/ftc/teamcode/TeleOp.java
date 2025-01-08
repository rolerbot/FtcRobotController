package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RobotFTC", group="Linear Opmode")
public class TeleOp extends GlobalScope
{
    private ElapsedTime runtime = new ElapsedTime();

    //private Lift SLider1, SLider2;

    public void runOpMode()
    {
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
            ActiuneAuto();
            BazaExt();
            ParkButton();
            //Cleste();
            Cleste();
            telemetry.update();
            telemetry.addData("Parcare", Parcare.getPosition());
            telemetry.addData("IntakeDr", IntakeDreapta.getPosition());
            telemetry.addData("BazaSt", BazaStanga.getPosition());
            telemetry.addData("BazaDr", BazaDreapta.getPosition());
            telemetry.addData("Rotire", ServoRotire.getPosition());
            sus.readValue();
            jos.readValue();
            if(sus.wasJustPressed())
                Parcare.setPosition(Parcare.getPosition() + 0.007);
            if(jos.wasJustPressed())
                Parcare.setPosition(Parcare.getPosition() - 0.007);
        }
    }
}