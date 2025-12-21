package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RobotFTC", group="Linear Opmode")
public class TeleOp extends GlobalScope
{

    public void runOpMode()
    {

        Initialise();
        MapControlerButtons();

        schimbator = 1.4 - schimbator;

        waitForStart();

        while (opModeIsActive())
        {
            MotorIn();
            telemetry.update();
        }
    }
}