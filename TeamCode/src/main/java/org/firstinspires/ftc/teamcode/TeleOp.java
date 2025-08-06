package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
            //Roteste();
            //OutakeRotire();
            ActiuneAuto();
            BazaExt();
            //ParkButton();
            Cleste();
            //Specimen();
            //Activeintake();
            telemetry.update();
            sus.readValue();
            jos.readValue();
            OuttakeSus2.readValue();
            OuttakeJos2.readValue();
            IntakeSus.readValue();
            IntakeJos.readValue();
            OutakeSus.readValue();
            OutakeJos.readValue();
            cLESTE1.readValue();
            cLESTE2.readValue();
            if(cLESTE1.wasJustPressed())
                ServoGhearaIntake.setPosition(ServoGhearaIntake.getPosition() + 0.001);
            if(cLESTE2.wasJustPressed())
                ServoGhearaIntake.setPosition(ServoGhearaIntake.getPosition() - 0.001);
            if(sus.wasJustPressed()) PivotIntake.setPosition(PivotIntake.getPosition() + 0.001);
            if(jos.wasJustPressed()) PivotIntake.setPosition(PivotIntake.getPosition() - 0.001);
            if(IntakeSus.wasJustPressed())
            {
                IntakeDreapta.setPosition(IntakeDreapta.getPosition() + 0.001);
                IntakeStanga.setPosition(IntakeStanga.getPosition() + 0.001);
            }
            if(IntakeJos.wasJustPressed())
            {
                IntakeDreapta.setPosition(IntakeDreapta.getPosition() - 0.001);
                IntakeStanga.setPosition(IntakeStanga.getPosition() - 0.001);
            }
            if(OutakeSus.wasJustPressed())
            {
                OutakeDreapta.setPosition(OutakeDreapta.getPosition() + 0.001);
            }
            if(OutakeJos.wasJustPressed())
            {
                OutakeDreapta.setPosition(OutakeDreapta.getPosition() - 0.001);
            }
            if(OuttakeSus2.wasJustPressed())
            {
                OutakeStanga.setPosition(OutakeStanga.getPosition() + 0.001);
            }
            if(OuttakeJos2.wasJustPressed())
            {
                OutakeStanga.setPosition(OutakeStanga.getPosition() - 0.001);
            }
            telemetry.addData("IntakeStanga:", IntakeStanga.getPosition());
            telemetry.addData("IntakeDr:", IntakeDreapta.getPosition());
            telemetry.addData("Otakedr:", OutakeDreapta.getPosition());
            telemetry.addData("OutakeSt:", OutakeStanga.getPosition());
            telemetry.addData("Pivot", PivotIntake.getPosition());
            telemetry.addData("Slider", SliderS.getCurrentPosition());
        }
    }
}