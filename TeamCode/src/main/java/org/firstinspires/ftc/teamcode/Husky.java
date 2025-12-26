package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// ID 1 - Green, Purple, Purple
// ID 2 - Purple, Green, Purple
// ID 3 - Purple, Purple, Green
// ID 4 - Blue
// ID 5 - Red
public class Husky implements Subsystem
{
    private HuskyLens huskyLens;
    private boolean allignPrepare = false;
    private boolean huskyRead = false;
    private int ID = 0;
    private boolean completeArtifact = false;
    public Color[] artifactOrder = new Color[3];
    private final TelemetryCustom telemetry;
    ButtonReader Allign;
    GamepadEx ct1;

    public Husky(TelemetryCustom tl, GamepadEx ct1)
    {
        this.ct1 = ct1;
        this.telemetry = tl;
    }
    public void LinkComponents(HardwareMap hardwareMap)
    {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
    }
    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        Allign = new ButtonReader(ct1, GamepadKeys.Button.LEFT_BUMPER);
    }
    public void Run() {ReadHusky();}
    private void ReadHusky()
    {
        telemetry.Log("ID:", GetID());
        if(!huskyRead)
        {
            HuskyLens.Block[] blocuri = huskyLens.blocks();
             for (int i = 0; i < blocuri.length; i++)
                if(blocuri[i].id != 0 && blocuri[i].id <= 3)
                {
                    ID = blocuri[i].id;
                    huskyRead = true;
                    break;
                }
        }
        if(huskyRead && !completeArtifact)
            CompleteColor();
    }
    public int GetID() {return ID;}
    private void CompleteColor()
    {
        completeArtifact = true;
        if(ID == 1)
        {
            artifactOrder[0] = Color.Green;
            artifactOrder[1] = Color.Purple;
            artifactOrder[2] = Color.Purple;
        }
        else if(ID == 2)
        {
            artifactOrder[0] = Color.Purple;
            artifactOrder[1] = Color.Green;
            artifactOrder[2] = Color.Purple;
        }
        else if(ID == 3)
        {
            artifactOrder[0] = Color.Purple;
            artifactOrder[1] = Color.Green;
            artifactOrder[2] = Color.Purple;
        }
        else
        {
            artifactOrder[0] = Color.Purple;
            artifactOrder[1] = Color.Purple;
            artifactOrder[2] = Color.Green;
        }
    }

    private void AutoAllign()
    {

    }

    private void AutoAllingnPrepare()
    {
        Allign.readValue();
        if(Allign.wasJustPressed())
            allignPrepare = true;
    }
}
