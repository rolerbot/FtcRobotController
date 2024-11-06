package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Robot16Aug", group="Linear Opmode")
public class TeleOpSlidere extends GlobalScopeSlidere
{
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode()
    {

        Initialise();

        waitForStart();

        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);
        IAMSPEED = new ButtonReader(ct1, GamepadKeys.Button.B);
        Vit = new ButtonReader(ct2, GamepadKeys.Button.Y);
        RotireStanga = new ButtonReader(ct2, GamepadKeys.Button.DPAD_LEFT);
        RotireDreapta = new ButtonReader(ct2, GamepadKeys.Button.DPAD_RIGHT);
        GhearaStanga = new TriggerReader(ct2, GamepadKeys.Trigger.LEFT_TRIGGER);
        GhearaDreapta = new TriggerReader(ct2, GamepadKeys.Trigger.RIGHT_TRIGGER);
        Launch = new ButtonReader(ct1, GamepadKeys.Button.X);
        VitezaPozitivaIntake = new ButtonReader(ct1, GamepadKeys.Button.A);
        IntakeUp = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        IntakeDown = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
        VitezaNegativaIntake = new ButtonReader(ct1, GamepadKeys.Button.Y);
        BratSus = new ButtonReader(ct2, GamepadKeys.Button.LEFT_BUMPER);
        BratJos = new ButtonReader(ct2, GamepadKeys.Button.RIGHT_BUMPER);

        while (opModeIsActive())
        {
            MiscareBaza();
            SliderExtend();
            SliderBaza();
            Intake();
            Roteste();
            Cleste();
            Brat();
            telemetry.update();
            telemetry.addData("IntakeCNT", pozitieActualaIntake);
        }
    }
}