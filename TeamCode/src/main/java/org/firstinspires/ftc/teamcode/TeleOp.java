package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RobotFTC", group="Linear Opmode")
public class TeleOp extends GlobalScope
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx imaginar = null;

    public void runOpMode()
    {
        imaginar = hardwareMap.get(DcMotorEx.class, "MotorulImaginar");
        Initialise();

        waitForStart();

        Controler();

        while (opModeIsActive())
        {

            MiscareBaza();
            SliderPoz2();
            SliderBaza();
            Roteste();
            OutakeRotire();
            ActiuneAuto();
            BazaExt();
            ParkButton();
            Cleste();
            Specimen();
            telemetry.update();
            telemetry.addData("OutakeSt", OutakeStanga.getPosition());
            telemetry.addData("OutakeDr", OutakeDreapta.getPosition());
            telemetry.addData("SliderSt", SliderS.getCurrentPosition());
            sus.readValue();
            jos.readValue();
            if(sus.wasJustPressed())
                OutakeStanga.setPosition(OutakeStanga.getPosition() + 0.007);
            if(jos.wasJustPressed())
                OutakeStanga.setPosition(OutakeStanga.getPosition() - 0.007);
            telemetry.addData("MotorulImaginar", imaginar.getCurrentPosition() );
            telemetry.addData("MotorFS", MotorFS.getCurrentPosition());
            telemetry.addData("MotorSD", MotorSD.getCurrentPosition());
            

        }
    }
}