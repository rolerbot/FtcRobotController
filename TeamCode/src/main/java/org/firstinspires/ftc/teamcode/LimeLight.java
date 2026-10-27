package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.List;

/**
 * Limelight: motivul (tag-urile 21 / 22 / 23 de pe obelisc -> ordinea bilelor) si tag-ul goal-ului
 * (20 albastru / 24 rosu) pentru corectia de heading (HeadingCameraCorrector).
 *
 * PIPELINE = 1: pipeline AprilTag nativ (ID filter 20-24, fara 3D). Da lista de tag-uri cu ID si tx.
 * PIPELINE = 0: vechiul pipeline Python (doar motivul, fara tx) - rezerva, merge exact ca inainte.
 * Se poate schimba live din Panels.
 */
@Configurable
public class LimeLight implements Subsystem {
    public static int PIPELINE = 1;
    private static final int PYTHON_PIPELINE = 0;

    private Limelight3A limelight;
    private int IdTag = 0;
    private final boolean isBlue;
    private GamepadEx ct2;
    private ButtonReader btn21, btn22, btn23;
    private int currentPipeline = -1;

    public Color[] artifactOrder = { Color.None, Color.None, Color.None };
    private boolean artifactOrderDetected = false;

    // tag-ul goal-ului alliantei, din ultimul cadru
    private boolean goalSeen = false;
    private double goalTx = 0, goalTy = 0;
    private double stalenessMs = 1e9;

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

        switchPipeline(PIPELINE);
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

        switchPipeline(PIPELINE);
        goalSeen = false;

        LLResult result = limelight.getLatestResult();
        if (result == null) return;
        stalenessMs = result.getStaleness();

        if (PIPELINE == PYTHON_PIPELINE) {
            // vechiul mod: doar motivul, din scriptul Python
            if (artifactOrderDetected) return;
            double[] pythonOut = result.getPythonOutput();
            if (pythonOut != null && pythonOut.length >= 2 && pythonOut[0] > 0.5) {
                int id = (int) pythonOut[1];
                if (id == 21 || id == 22 || id == 23) {
                    SetDetectedId(id);
                }
            }
            return;
        }

        // pipeline AprilTag nativ: fiecare tag cu ID-ul si tx-ul lui
        if (!result.isValid()) return;
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return;
        int goalId = isBlue ? 20 : 24;
        for (LLResultTypes.FiducialResult t : tags) {
            int id = t.getFiducialId();
            if (!artifactOrderDetected && (id == 21 || id == 22 || id == 23)) {
                SetDetectedId(id);
            } else if (id == goalId) {
                goalSeen = true;
                goalTx = t.getTargetXDegrees();
                goalTy = t.getTargetYDegrees();
            }
        }
    }

    // ---------- tag-ul goal-ului (pentru corectia de heading) ----------

    /** true daca in ultimul cadru se vede tag-ul goal-ului alliantei (doar pe pipeline-ul AprilTag). */
    public boolean HasGoalTag() { return goalSeen; }

    /** Unghiul orizontal pana la tag-ul goal-ului (grade, + = spre dreapta). */
    public double GetGoalTx() { return goalTx; }

    /** Unghiul vertical pana la tag-ul goal-ului (grade, + = in sus). */
    public double GetGoalTy() { return goalTy; }

    /** Cat de vechi e ultimul rezultat de la camera (ms). */
    public double GetStalenessMs() { return stalenessMs; }

    // ---------- motivul ----------

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
