package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private int IdArranged = 0;
    private int IdTeam = 0;
    private boolean completeArtifact = false;
    public Color[] artifactOrder = {Color.None, Color.None, Color.None};
    String[] arrCol = {"GPP", "PGP", "PPG"};

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
        if(!huskyRead)
        {
            HuskyLens.Block[] blocuri = huskyLens.blocks();
             for (int i = 0; i < blocuri.length; i++)
                if(blocuri[i].id != 0 && blocuri[i].id <= 3)
                {
                    IdArranged = blocuri[i].id;
                    huskyRead = true;
                    telemetry.Log("ID:", GetID());
                    break;
                }
        }
        if(huskyRead && !completeArtifact)
            CompleteColor(IdArranged);
    }
    public int GetID() {return IdArranged;}
    private void CompleteColor(int id)
    {
        completeArtifact = true;
        for(int i = 0; i < arrCol[id - 1].length(); i++)
        {
            char c = arrCol[id - 1].charAt(i);
            artifactOrder[i] = CharToColor(c);
        }
    }

    private Color CharToColor(char c)
    {
        switch(c)
        {
            case 'G':
                return Color.Green;
            case 'P':
                return Color.Purple;
            default:
                return Color.None;
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
