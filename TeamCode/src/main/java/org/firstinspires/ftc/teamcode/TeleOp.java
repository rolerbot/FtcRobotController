package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.parts.Lift;
import org.firstinspires.ftc.teamcode.parts.PrindereAuto;

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
            SliderAutoPoz();
            ActiuneAuto();
            BazaExt();
            //SlidePoz();
            Cleste();
            //Intake();
            //Outake();
            telemetry.update();
            telemetry.addData("Rotire", ServoRotire.getPosition());
            telemetry.addData("SLider", SliderS.getCurrentPosition());
            telemetry.addData("GhearaOutake", ServoGhearaIntake.getPosition());
            telemetry.addData("pozitieOutake", pozitieOutake);
            sus.readValue();
            jos.readValue();
            if(sus.wasJustPressed())
                ServoGhearaIntake.setPosition(ServoGhearaIntake.getPosition() + 0.001);
            if(jos.wasJustPressed())
                ServoGhearaIntake.setPosition(ServoGhearaIntake.getPosition() - 0.001);
        }
    }
}