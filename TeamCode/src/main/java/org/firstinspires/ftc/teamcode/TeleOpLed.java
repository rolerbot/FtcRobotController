package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class TeleOpLed extends OpMode
{

    private LedArtifact ledArtifact;

    @Override
    public void init()
    {
        ledArtifact = new LedArtifact(hardwareMap);
    }

    @Override
    public void loop()
    {
        if(gamepad1.aWasPressed())
        {
            ledArtifact.SetGreenLed(true);
            ledArtifact.SetRedLed(false);
        }

        else if (gamepad1.bWasPressed())
        {
            ledArtifact.SetGreenLed(false);
            ledArtifact.SetRedLed(true);
        }
        else if(gamepad1.xWasPressed())
        {
            ledArtifact.SetGreenLed(true);
            ledArtifact.SetRedLed(true);
        }
        else if(gamepad1.yWasPressed())
        {
            ledArtifact.StopLeds();
        }

    }
}
