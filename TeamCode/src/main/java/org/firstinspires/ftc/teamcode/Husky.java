package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

// ID 1 - Green, Purple, Purple
// ID 2 - Purple, Green, Purple
// ID 3 - Purple, Purple, Green
// ID 4 - Blue
// ID 5 - Red
public class Husky implements Subsystem
{
    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1;
    private boolean huskyRead = false;
    private ElapsedTime runtime = new ElapsedTime();
    private int ID = 0;
    private boolean completeArtifact = false;
    public Color[] artifactOrder = new Color[3];

    public void LinkComponents(HardwareMap hardwareMap)
    {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
    }
    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

    }
    public void Run()
    {
        ReadHusky();
    }

    private void ReadHusky()
    {
        if (runtime.seconds() >= READ_PERIOD && !huskyRead)
        {
            runtime.reset();
        }
        else if(!huskyRead)
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
        {
            CompleteColor();
        }
    }

    public int GetID()
    {
        return ID;
    }

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
}
