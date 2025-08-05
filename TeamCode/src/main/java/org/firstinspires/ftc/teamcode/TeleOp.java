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
            //ActiuneAuto();
            BazaExt();
            ParkButton();
            Cleste();
            Specimen();
            Activeintake();
            telemetry.update();
            sus.readValue();
            jos.readValue();
            GasirePozitii(sus, jos, PivotIntake);
            GasirePozitii(IntakeSus, IntakeJos, IntakeStanga);
            GasirePozitii(IntakeSus, IntakeJos, IntakeDreapta);
            GasirePozitii(OutakeSus, OutakeJos, OutakeDreapta);
            GasirePozitii(OutakeSus, OutakeJos, OutakeStanga);
            telemetry.addData("IntakeStanga:", IntakeStanga.getPosition());
            telemetry.addData("IntakeDr:", IntakeDreapta.getPosition());
            telemetry.addData("Otakedr:", OutakeDreapta.getPosition());
            telemetry.addData("OutakeSt:", OutakeStanga.getPosition());
            telemetry.addData("Pivot", PivotIntake.getPosition());
        }
    }
}