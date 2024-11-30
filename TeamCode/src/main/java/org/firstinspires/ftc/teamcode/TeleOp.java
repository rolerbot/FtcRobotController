package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.Servo;
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
        BazaDreapta.setPosition(0.02);
        BazaStanga.setPosition(0.02);
        IntakeStanga.setPosition(0);
        IntakeDreapta.setPosition(0);
        OutakeStanga.setPosition(0.5);
        OutakeDreapta.setPosition(0.5);
        ServoGhearaIntake.setPosition(0);
        Viteza  = new ButtonReader(ct1, GamepadKeys.Button.B);
        IntakeSus = new ButtonReader(ct2, GamepadKeys.Button.DPAD_UP);
        IntakeJos = new ButtonReader(ct2, GamepadKeys.Button.DPAD_DOWN);
        GhearaIntake = new TriggerReader(ct2, GamepadKeys.Trigger.LEFT_TRIGGER);
        GhearaOutake = new TriggerReader(ct2, GamepadKeys.Trigger.RIGHT_TRIGGER);
        RotireStanga = new ButtonReader(ct2, GamepadKeys.Button.DPAD_LEFT);
        RotireDreapta = new ButtonReader(ct2, GamepadKeys.Button.DPAD_RIGHT);

        while (opModeIsActive())
        {
            MiscareBaza();
            SliderExtend();
            SliderBaza();
            Roteste();
            Cleste();
            Intake();
            telemetry.update();
            telemetry.addData("Stanga ", BazaStanga.getPosition());
            telemetry.addData("Dreapta ", BazaDreapta.getPosition());
            telemetry.addData("Gheara", ServoGhearaIntake.getPosition());
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