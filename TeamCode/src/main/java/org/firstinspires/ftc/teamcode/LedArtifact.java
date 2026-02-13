package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class LedArtifact
{
    private LED ledGreen;
    private LED ledRed;

    private void init(HardwareMap hardwareMap)
    {
        ledRed = hardwareMap.get(LED.class, "ledRed");
        ledGreen = hardwareMap.get(LED.class, "ledGreen");
    }

    public void SetRedLed(boolean isOn)
    {
        if (isOn) ledRed.on();
        else ledRed.off();
    }

    public void SetGreenLed(boolean isOn)
    {
        if (isOn) ledGreen.on();
        else ledGreen.off();
    }

    public void StopLeds()
    {
        ledRed.off();
        ledGreen.off();
    }

     public LedArtifact(HardwareMap hardwareMap)
     {
         init(hardwareMap);
     }
}
