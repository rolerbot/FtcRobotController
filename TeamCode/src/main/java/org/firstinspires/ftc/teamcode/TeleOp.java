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
        //SLider1 = new Lift(hardwareMap);
        //SLider2 = new Lift(hardwareMap);

        while (opModeIsActive())
        {
            //SLider1.pressedButton(gamepad2.y);
            //SLider2.PressedButton(gamepad2.a);
            MiscareBaza();
            //SliderPoz();
            //SliderExtend();
            SliderPoz2();
            SliderBaza();
            Roteste();
            OutakeRotire();
            ActiuneAuto();
            BazaExt();
            //Cleste();
            Cleste();
            telemetry.update();
            telemetry.addData("INtakeSt", IntakeStanga.getPosition());
            telemetry.addData("IntakeDr", IntakeDreapta.getPosition());
            telemetry.addData("BazaSt", BazaStanga.getPosition());
            telemetry.addData("BazaDr", BazaDreapta.getPosition());
            sus.readValue();
            jos.readValue();
            if(sus.wasJustPressed())
                OutakeStanga.setPosition(OutakeStanga.getPosition() + 0.015);
            if(jos.wasJustPressed())
                OutakeStanga.setPosition(OutakeStanga.getPosition() - 0.015);
        }
    }
}