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
    private boolean huskyRead = false;
    private int IdArranged = 0;
    private boolean completeArtifact = false;
    private int tagX = 0;
    private int tagY = 0;
    private int IdTeam = 4; // Blue, 5- Red
    public Color[] artifactOrder = {Color.None, Color.None, Color.None};
    String[] arrCol = {"GPP", "PGP", "PPG"};

    private final TelemetryCustom telemetry;
    ButtonReader AlignButton;
    GamepadEx ct1;


    // Constructor with all parameters
    public Husky(TelemetryCustom tl, GamepadEx ct1)
    {
        this.ct1 = ct1;
        this.telemetry = tl;
    }

    // Constructor with just telemetry (for autonomous)
    public Husky(TelemetryCustom tl)
    {
        this.telemetry = tl;
        this.ct1 = null;
    }

    public void LinkComponents(HardwareMap hardwareMap)
    {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
    }
    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // Only initialize button if gamepad exists (teleop mode)
        if (ct1 != null)
        {
            AlignButton = new ButtonReader(ct1, GamepadKeys.Button.RIGHT_BUMPER);
        }
    }
    public void Run()
    {
        HuskyLens.Block[] blocks = huskyLens.blocks();

        // Procesează datele pentru aranjament (ID 1-3)
        ReadHusky(blocks);

    }
    private void ReadHusky(HuskyLens.Block[] blocuri)
    {
        if(!huskyRead)
        {
            for (HuskyLens.Block block : blocuri)
            {
                if(block.id != 0 && block.id <= 3)
                {
                    IdArranged = block.id;
                    huskyRead = true;
                    telemetry.Log("Intaltime", block.height);
                    telemetry.Log("X:", block.x);
                    telemetry.Log("Y:", block.y);
                    telemetry.Log("String", block.toString());
                    telemetry.Log("ID:", GetID());
                    break;
                }
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
    public void SetTeamId(int teamId)
    {
        if (teamId == 4 || teamId == 5)
            this.IdTeam = teamId;
    }
    public int GetTeamId() {return IdTeam;}
    public int GetTagX() {return tagX;}
    public int GetTagY() {return tagY;}
}
