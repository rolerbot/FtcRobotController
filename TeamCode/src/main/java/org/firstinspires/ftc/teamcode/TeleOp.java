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
            sus.readValue();
            jos.readValue();
            if(sus.wasJustPressed())
                OutakeDreapta.setPosition(OutakeDreapta.getPosition() + 0.001);
            if(jos.wasJustPressed())
                OutakeDreapta.setPosition(OutakeDreapta.getPosition() - 0.001);
            telemetry.addData("Parcare", Parcare.getPosition());
            telemetry.addData("OutakeDr", OutakeDreapta.getPosition());
        }
    }
}