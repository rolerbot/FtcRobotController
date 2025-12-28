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
    // Enum pentru stările auto-alinieării
    private enum AlignmentState
    {
        IDLE,               // Nu face nimic
        ROTATING,           // Se rotește pentru a pune tag-ul în dreapta
        ADJUSTING_DISTANCE, // Se apropie/depărtează pentru distanța țintă
        COMPLETED           // Aliniere completată
    }

    private HuskyLens huskyLens;
    private boolean huskyRead = false;
    private int IdArranged = 0;
    private boolean tagDetected = false;
    private boolean completeArtifact = false;
    private double currentDistance = -1;
    private int tagX = 0;
    private int tagY = 0;
    private int tagWidth = 0;
    private int tagHeight = 0;
    private int IdTeam = 4; // Blue, 5- Red
    private static final double TAG_SIZE_CM = 16.5; // Mărimea reală a AprilTag-ului (16.5cm x 16.5cm)
    private static final double FOCAL_LENGTH = 230.0; // Distanța focală a camerei (trebuie calibrată!)
    public Color[] artifactOrder = {Color.None, Color.None, Color.None};
    String[] arrCol = {"GPP", "PGP", "PPG"};

    private final TelemetryCustom telemetry;
    private Drivetrain drivetrain;
    ButtonReader AlignButton;
    GamepadEx ct1;

    // Variabile pentru auto-aliniere
    private boolean isAligning = false;
    private AlignmentState alignmentState = AlignmentState.IDLE;
    private static final double TARGET_DISTANCE = 120.0; // cm
    private static final double DISTANCE_TOLERANCE_MIN = 100.0; // cm
    private static final double DISTANCE_TOLERANCE_MAX = 140.0; // cm
    private static final int CAMERA_CENTER_X = 160; // Centrul camerei (rezoluție 320x240)
    private static final int CAMERA_RIGHT_THRESHOLD = 100; // Pragul pentru partea dreaptă
    private static final double ROTATION_SPEED = 0.5;
    private static final double APPROACH_SPEED = 0.7;

    public Husky(TelemetryCustom tl, GamepadEx ct1, Drivetrain drivetrain)
    {
        this.ct1 = ct1;
        this.telemetry = tl;
        this.drivetrain = drivetrain;
    }

    public void LinkComponents(HardwareMap hardwareMap)
    {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
    }
    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        AlignButton = new ButtonReader(ct1, GamepadKeys.Button.RIGHT_BUMPER);
    }
    public void Run()
    {
        HuskyLens.Block[] blocks = huskyLens.blocks();

        // Procesează datele pentru aranjament (ID 1-3)
        ReadHusky(blocks);

        // Procesează datele pentru distanță (tag-ul echipei)
        UpdateTagData(blocks);

        // Verifică butonul pentru auto-aliniere
        CheckAlignmentButton();

        // Execută auto-aliniearea dacă este activată
        if (isAligning)
        {
            PerformAlignment();
        }

        // Afișează telemetria
        DisplayTelemetry();
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

    public boolean IsTagDetected()
    {
        return tagDetected;
    }

    private void UpdateTagData(HuskyLens.Block[] blocks)
    {
        tagDetected = false;

        for (HuskyLens.Block block : blocks)
        {
            if (block.id == IdTeam)
            {
                tagDetected = true;
                tagX = block.x;
                tagY = block.y;
                tagWidth = block.width;
                tagHeight = block.height;

                currentDistance = CalculateDistance(tagWidth);
                break;
            }
        }
        if (!tagDetected)
        {
            currentDistance = -1;
        }
    }

    public static double CalibrateFocalLength(double knownDistanceCm, int measuredWidthPixels)
    {
        return (measuredWidthPixels * knownDistanceCm) / TAG_SIZE_CM;
    }

    private double CalculateDistance(int widthPixels)
    {
        if (widthPixels <= 0) return -1;

        return (TAG_SIZE_CM * FOCAL_LENGTH) / widthPixels;
    }
    private void DisplayTelemetry()
    {
        telemetry.Log("=== HUSKY DISTANCE ===", "");

        String teamName = (IdTeam == 4) ? "Blue" : "Red";
        telemetry.Log("Team ID", IdTeam + " (" + teamName + ")");

        telemetry.Log("Tag Detected", tagDetected ? "YES" : "NO");

        if (tagDetected)
        {
            telemetry.Log("Tag Position X", tagX);
            telemetry.Log("Tag Position Y", tagY);
            telemetry.Log("Tag Width (px)", tagWidth);
            telemetry.Log("Tag Height (px)", tagHeight);
            telemetry.Log("Distance (cm)", String.format("%.2f", currentDistance));
        }
        else
            telemetry.Log("Distance", "N/A");

        // Status auto-aliniere
        if (isAligning)
        {
            telemetry.Log("=== AUTO-ALIGN ===", "ACTIVE");
            telemetry.Log("State", alignmentState.toString());
        }
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

    public void CalibrateMode(double knownDistanceCm)
    {
        if (tagDetected)
        {
            double calculatedFocalLength = CalibrateFocalLength(knownDistanceCm, tagWidth);
            telemetry.Log("=== CALIBRATION ===", "");
            telemetry.Log("Known Distance (cm)", knownDistanceCm);
            telemetry.Log("Measured Width (px)", tagWidth);
            telemetry.Log("Calculated Focal Length", String.format("%.2f", calculatedFocalLength));
            telemetry.Log("INSTRUCTION", "Update FOCAL_LENGTH constant to " + String.format("%.2f", calculatedFocalLength));
        }
        else
        {
            telemetry.Log("CALIBRATION", "No tag detected!");
        }
    }
    private void CheckAlignmentButton()
    {
        AlignButton.readValue();
        if (AlignButton.wasJustPressed())
        {
            if (!isAligning)
            {
                // Începe auto-aliniearea
                isAligning = true;
                alignmentState = AlignmentState.ROTATING;
                telemetry.Log("Auto-Align", "STARTED");
            }
            else
            {
                // Oprește auto-aliniearea
                StopAlignment();
            }
        }
    }
    private void PerformAlignment()
    {
        if (!tagDetected)
        {
            telemetry.Log("Auto-Align", "NO TAG DETECTED!");
            StopAlignment(); // Folosește StopAlignment pentru a reseta complet
            return;
        }

        switch (alignmentState)
        {
            case ROTATING:
                AlignHorizontally();
                break;
            case ADJUSTING_DISTANCE:
                AdjustDistance();
                break;
            case COMPLETED:
                // Resetează complet și returnează controlul la manual
                StopAlignment();
                telemetry.Log("Auto-Align", "COMPLETED!");
                break;
            case IDLE:
                // Nu ar trebui să ajungă aici, dar pentru siguranță
                isAligning = false;
                break;
        }
    }
    private void AlignHorizontally()
    {
        telemetry.Log("Alignment State", "ROTATING");

        if (tagX > CAMERA_RIGHT_THRESHOLD)
        {
            // Tag-ul este deja în dreapta, trecem la ajustarea distanței
            alignmentState = AlignmentState.ADJUSTING_DISTANCE;
            drivetrain.Stop();
            telemetry.Log("Rotation", "COMPLETED");
        }
        else if (tagX < CAMERA_CENTER_X)
        {
            // Tag-ul este în stânga, rotește la dreapta
            drivetrain.Rotate(ROTATION_SPEED);
            telemetry.Log("Rotating", "RIGHT");
        }
        else
        {
            // Tag-ul este aproape de zona dreaptă
            drivetrain.Rotate(ROTATION_SPEED * 0.5); // Rotație mai lentă
            telemetry.Log("Rotating", "SLOW RIGHT");
        }
    }
    private void AdjustDistance()
    {
        telemetry.Log("Alignment State", "ADJUSTING DISTANCE");
        telemetry.Log("Current Distance", String.format("%.2f cm", currentDistance));
        telemetry.Log("Target Distance", String.format("%.2f cm", TARGET_DISTANCE));

        if (currentDistance < 0)
        {
            // Nu putem calcula distanța
            drivetrain.Stop();
            StopAlignment();
            telemetry.Log("Error", "Cannot calculate distance!");
            return;
        }

        if (currentDistance >= DISTANCE_TOLERANCE_MIN && currentDistance <= DISTANCE_TOLERANCE_MAX)
        {
            // Distanța este OK, oprește
            alignmentState = AlignmentState.COMPLETED;
            drivetrain.Stop();
        }
        else if (currentDistance < DISTANCE_TOLERANCE_MIN)
        {
            // Prea aproape, dă-te înapoi
            drivetrain.MoveForward(-APPROACH_SPEED);
            telemetry.Log("Moving", "BACKWARD");
        }
        else
        {
            // Prea departe, mergi înainte
            drivetrain.MoveForward(APPROACH_SPEED);
            telemetry.Log("Moving", "FORWARD");
        }
    }

    private void StopAlignment()
    {
        isAligning = false;
        alignmentState = AlignmentState.IDLE;
        drivetrain.Stop();
        telemetry.Log("Auto-Align", "STOPPED");
    }
    public void SetTeamId(int teamId)
    {
        if (teamId == 4 || teamId == 5)
            this.IdTeam = teamId;
    }
    public int GetTeamId() {return IdTeam;}

    public double GetDistance() {return currentDistance;}
    public double GetDistanceInches()
    {
        if (currentDistance < 0) return -1;
        return currentDistance / 2.54;
    }
    public int GetTagX() {return tagX;}
    public int GetTagY() {return tagY;}
}
