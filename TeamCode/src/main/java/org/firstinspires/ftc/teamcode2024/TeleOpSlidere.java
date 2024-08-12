package org.firstinspires.ftc.teamcode2024;

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

        Intake.setPosition(0);
        ServoRotire.setPosition(0.5);//0.1105
        ServoBrat.setPosition(0.05);//0.19
        /*while(!RevButon.isPressed())
        {
            SliderStanga.setPower(-1);
            SliderDreapta.setPower(-1);
        }*/
        sleep(850);
        ServoBrat.setPosition(0.17);//0.19
        ServoGhearaStanga.setPosition(0.65);
        ServoGhearaDreapta.setPosition(0.3875);//0.6
        ServoStanga.setPosition(0.17);//0.185
        ServoDreapta.setPosition(0);//0.015

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
            WeGottaMove();
            WeGottaExtend();
            //SlideVit();
            Intake();
            Roteste();
            SenzoriCuloare();
            //SenzorSiCleste();
            WeGottaLunchDrone();
            Cleste();
            Brat();
            //SRotire 190 grade poz= 0.11
            telemetry.update();
            /*if (gamepad1.dpad_up)
                ServoRotire.setPosition(ServoRotire.getPosition() + 0.01);
            if(gamepad1.dpad_down)
                ServoRotire.setPosition(ServoRotire.getPosition() - 0.01);
            telemetry.addData("Rotire", ServoRotire.getPosition());*/
            telemetry.addData("Distanta Dreapta ", SenzorDreapta.getDistance(DistanceUnit.MM));
            telemetry.addData("Distanta Stanga " , SenzorStanga.getDistance(DistanceUnit.MM));
        }
    }
}
