package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.parts.Lift;
import org.firstinspires.ftc.teamcode.parts.PrindereAuto;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RobotFTC", group="Linear Opmode")
public class TeleOp extends GlobalScope
{
    private ElapsedTime runtime = new ElapsedTime();

    private Lift SLider1, SLider2;

    public void runOpMode()
    {
        Initialise();

        waitForStart();

        Controler();
        SLider1 = new Lift(hardwareMap);
        //SLider2 = new Lift(hardwareMap);

        while (opModeIsActive())
        {
            SLider1.pressedButton(gamepad2.y);
            //SLider2.PressedButton(gamepad2.a);
            MiscareBaza();
            //SliderExtend();
            SliderBaza();
            Roteste();
            OutakeRotire();
            ActiuneAuto3();
            BazaExt();
            Cleste();
            telemetry.update();
            telemetry.addData("Rotire", ServoRotire.getPosition());
            telemetry.addData("SLider", SliderS.getCurrentPosition());
            telemetry.addData("GhearaOutake", ServoGhearaOutake.getPosition());
            telemetry.addData("pozitieOutake", pozitieOutake);
            //telemetry.addData("timp", time);
            sus.readValue();
            jos.readValue();
            //if(sus.wasJustPressed())
                //ServoGhearaOutake.setPosition(ServoGhearaOutake.getPosition() + 0.001);
            //if(jos.wasJustPressed())
                //ServoGhearaOutake.setPosition(ServoGhearaOutake.getPosition() - 0.001);
        }
    }
}