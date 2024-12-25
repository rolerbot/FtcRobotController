package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.parts.Baza;
import org.firstinspires.ftc.teamcode.parts.Lift;
import org.firstinspires.ftc.teamcode.parts.Outake;
import org.firstinspires.ftc.teamcode.parts.PrindereAuto;
import org.firstinspires.ftc.teamcode.parts.Intake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RobotFTC2", group="Linear Opmode")
public class TeleOp2 extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private Lift SlidereSus, SlidereJos;
    private PrindereAuto IntakeOutake;
    private Baza Vit;
    private Outake Out;
    private Intake In;

    public void runOpMode()
    {

        waitForStart();

        IntakeOutake = new PrindereAuto(hardwareMap);
        SlidereSus = new Lift(hardwareMap);
        SlidereJos = new Lift(hardwareMap);
        Vit = new Baza(hardwareMap);
        Out = new Outake(hardwareMap);
        In = new Intake(hardwareMap);

        while (opModeIsActive())
        {
            IntakeOutake.ActiuneAuto(gamepad2.dpad_up, gamepad2.dpad_down, gamepad1.dpad_up, gamepad1.dpad_down);
            SlidereSus.pressedButton(gamepad2.y);
            SlidereJos.PressedButton(gamepad2.a);
            Vit.MiscareBaza(gamepad1.b, gamepad1);
            Out.OutakeMiscare(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.right_trigger, gamepad1);
        }
    }
}