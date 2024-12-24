package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.parts.Lift;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RobotFTC", group="Linear Opmode")
public class TeleOp extends GlobalScope
{
    private ElapsedTime runtime = new ElapsedTime();

    private Lift lift;

    public void runOpMode()
    {
        lift = new Lift(hardwareMap);
        lift.pressedButton(gamepad1.a);

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