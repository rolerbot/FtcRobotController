package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

/**
 * Limelight folosit DOAR pentru motiv (tag-urile 21 / 22 / 23 de pe obelisc) -> ordinea bilelor.
 *
 * Pozitia robotului, distanta pana la goal si ochirea turelei vin acum din RobotPinpoint,
 * nu din camera. Pipeline-urile de goal (1 = rosu, 2 = albastru) nu mai sunt folosite.
 */
public class LimeLight implements Subsystem {
    private static final int MOTIF_PIPELINE = 0;

    private Limelight3A limelight;
    private int IdTag = 0;
    private final boolean isBlue;
    private GamepadEx ct2;
    private ButtonReader btn21, btn22, btn23;
    private int currentPipeline = -1;

    public Color[] artifactOrder = { Color.None, Color.None, Color.None };
    private boolean artifactOrderDetected = false;

    /** isAuto ramane in constructor ca sa nu se schimbe apelurile existente. */
    public LimeLight(boolean IsBlue, boolean isAuto) {
        this.isBlue = IsBlue;
    }

    public LimeLight(boolean IsBlue, boolean isAuto, GamepadEx ct2) {
        this.isBlue = IsBlue;
        this.ct2 = ct2;
    }

    public boolean IsBlue() {
        return isBlue;
    }

    public void LinkComponents(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void Initialize(HardwareMap hardwareMap) {
        LinkComponents(hardwareMap);

        limelight.setPollRateHz(125);
        limelight.start();

        if (ct2 != null) {
            btn21 = new ButtonReader(ct2, GamepadKeys.Button.Y);
            btn22 = new ButtonReader(ct2, GamepadKeys.Button.X);
            btn23 = new ButtonReader(ct2, GamepadKeys.Button.A);
        }

        switchPipeline(MOTIF_PIPELINE);
    }

    public void Run() {
        if (ct2 != null) {
            btn21.readValue();
            btn22.readValue();
            btn23.readValue();

            if (btn21.wasJustPressed()) {
                SetDetectedId(21);
            } else if (btn22.wasJustPressed()) {
                SetDetectedId(22);
            } else if (btn23.wasJustPressed()) {
                SetDetectedId(23);
            }
        }

        if (artifactOrderDetected)
            return; // motivul e cunoscut, nu mai citim camera

        switchPipeline(MOTIF_PIPELINE);

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            double[] pythonOut = result.getPythonOutput();
            if (pythonOut != null && pythonOut.length >= 2 && pythonOut[0] > 0.5) {
                int id = (int) pythonOut[1];
                if (id == 21 || id == 22 || id == 23) {
                    SetDetectedId(id);
                }
            }
        }
    }

    private void SetDetectedId(int id) {
        this.IdTag = id;
        SetOrder(id);
        this.artifactOrderDetected = true;
    }

    private void switchPipeline(int p) {
        if (currentPipeline != p) {
            limelight.pipelineSwitch(p);
            currentPipeline = p;
        }
    }

    /** Porneste din nou cautarea motivului. */
    public void ResetArtifactDetection() {
        artifactOrderDetected = false;
        IdTag = 0;
        artifactOrder[0] = Color.None;
        artifactOrder[1] = Color.None;
        artifactOrder[2] = Color.None;
    }

    /** Opreste cautarea si presupune motivul 21 (pentru timeout in autonomie). */
    public void SkipArtifactDetection() {
        artifactOrderDetected = true;
        SetOrder(21);
    }

    private void SetOrder(int id) {
        if (id == 21) {
            artifactOrder[0] = Color.Green;
            artifactOrder[1] = Color.Purple;
            artifactOrder[2] = Color.Purple;
            return;
        }
        if (id == 22) {
            artifactOrder[0] = Color.Purple;
            artifactOrder[1] = Color.Green;
            artifactOrder[2] = Color.Purple;
            return;
        }
        if (id == 23) {
            artifactOrder[0] = Color.Purple;
            artifactOrder[1] = Color.Purple;
            artifactOrder[2] = Color.Green;
        }
    }

    public int GetID() {
        return IdTag;
    }

    public boolean IsArtifactOrderDetected() {
        return artifactOrderDetected;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}
