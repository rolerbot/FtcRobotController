package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Robot16Aug", group="Linear Opmode")
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
            Cleste();
            Intake();
            Outake();
            telemetry.update();
            telemetry.addData("Stanga ", Baza.Stanga.getPosition());
            telemetry.addData("Dreapta ", Baza.Dreapta.getPosition());
            telemetry.addData("Gheara", Gheara.Intake.getPosition());
            telemetry.addData("inst", Intake.Stanga.getPosition());
            telemetry.addData("indr", Intake.Dreapta.getPosition());
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