package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.util.ElapsedTime;

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


        while (opModeIsActive())
        {
            MiscareBaza();
            SliderExtend();
            /**SliderBaza();
            Intake();
            Roteste();
            Cleste();
            Brat();*/
            telemetry.update();
        }
    }
}