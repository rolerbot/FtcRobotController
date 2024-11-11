package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Robot16Aug", group="Linear Opmode")
public class TeleOp extends GlobalScope
{
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode()
    {

        Initialise();

        waitForStart();

        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);
        BazaDreapta.setPosition(0);
        BazaStanga.setPosition(0);
        Intake.setPosition(0);
        ServoGhearaIntake.setPosition(0);
        ServoGhearaOutake.setPosition(0);
        Viteza  = new ButtonReader(ct1, GamepadKeys.Button.B);
        IntakeSus = new ButtonReader(ct2, GamepadKeys.Button.DPAD_UP);
        IntakeJos = new ButtonReader(ct2, GamepadKeys.Button.DPAD_DOWN);

        while (opModeIsActive())
        {
            MiscareBaza();
            SliderExtend();
            SliderBaza();
            Intake();
            //Roteste();
            //Cleste();
            //Brat();
            telemetry.update();
            telemetry.addData("Stanga ", BazaStanga.getPosition());
            telemetry.addData("Dreapta ", BazaDreapta.getPosition());
            telemetry.addData("PozitieBrat", Intake.getPosition());
        }
    }
}