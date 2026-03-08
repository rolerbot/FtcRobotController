package org.firstinspires.ftc.teamcode;

// ============================================================
//  SHOOTER TUNING SYSTEM - TABLE BASED
//
//  ct2 CONTROLS:
//  DPAD UP / DOWN        → navigate rows in table
//  DPAD LEFT / RIGHT     → adjust RPM at selected row (± step)
//  LEFT BUMPER / RIGHT BUMPER → adjust hood at selected row (± step)
//  B                     → cycle step size (50, 10, 1, 0.1, 0.001)
//  X                     → add new row using current limelight distance
//  LEFT_STICK_BUTTON     → save current RPM + hood into selected row
//  Y                     → test shoot (feeds one ball at current row values)
// ============================================================

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class ShooterTuningClass {

    // ── Data ────────────────────────────────────────────────
    public static class ShotParameters {
        public double distanceCm;
        public double rpm;
        public double hoodAngle;

        public ShotParameters(double distanceCm, double rpm, double hoodAngle) {
            this.distanceCm = distanceCm;
            this.rpm = rpm;
            this.hoodAngle = hoodAngle;
        }
    }

    // ── Default table (your previous tuned values as starting point) ─
    // Edit these to match your last known good values before a session
    private final List<ShotParameters> table = new ArrayList<>();

    private void LoadDefaultTable() {
        table.clear();
        table.add(new ShotParameters(120, 1110, 0.550));
        table.add(new ShotParameters(130, 1130, 0.555));
        table.add(new ShotParameters(150, 1160, 0.55));
        table.add(new ShotParameters(175, 1200, 0.565));
        table.add(new ShotParameters(200, 1230, 0.540));
        table.add(new ShotParameters(230, 1280, 0.556));
        table.add(new ShotParameters(260, 1300, 0.545));
        table.add(new ShotParameters(300, 1450, 0.565));
        table.add(new ShotParameters(340, 1550, 0.57));
        table.add(new ShotParameters(370, 1550, 0.59));
        SortTable();
    }

    // ── Interpolation (used by Shooter during match) ─────────
    public ShotParameters GetShotParams(double distanceCm) {
        if (table.isEmpty())
            return new ShotParameters(distanceCm, 1300, 0.51);

        // Clamp to bounds
        if (distanceCm <= table.get(0).distanceCm)
            return table.get(0);
        if (distanceCm >= table.get(table.size() - 1).distanceCm)
            return table.get(table.size() - 1);

        // Find surrounding rows and interpolate
        for (int i = 0; i < table.size() - 1; i++) {
            ShotParameters a = table.get(i);
            ShotParameters b = table.get(i + 1);
            if (distanceCm >= a.distanceCm && distanceCm <= b.distanceCm) {
                double t = (distanceCm - a.distanceCm) / (b.distanceCm - a.distanceCm);
                return new ShotParameters(
                        distanceCm,
                        Lerp(a.rpm, b.rpm, t),
                        Lerp(a.hoodAngle, b.hoodAngle, t));
            }
        }

        return table.get(table.size() - 1);
    }

    private double Lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private void SortTable() {
        Collections.sort(table, new Comparator<ShotParameters>() {
            @Override
            public int compare(ShotParameters a, ShotParameters b) {
                return Double.compare(a.distanceCm, b.distanceCm);
            }
        });
    }

    // ── Tuning State ─────────────────────────────────────────
    private int selectedRow = 0;
    private final double[] stepSizes = { 50, 10, 1, 0.1, 0.01, 0.001, 0.0001 };
    private int stepIndex = 1; // default step = 10

    private final GamepadEx ct2;
    private final TelemetryCustom telemetry;
    private final LimeLight limelight;

    private ButtonReader DpadUp, DpadDown, DpadLeft, DpadRight;
    private ButtonReader BumpLeft, BumpRight;
    private ButtonReader BButton, XButton, YButton;
    private ButtonReader SaveButton; // LEFT_STICK_BUTTON

    private boolean testShootRequested = false;
    private final ElapsedTime testShootTimer = new ElapsedTime();
    private boolean testShootActive = false;

    // ── Constructor ──────────────────────────────────────────
    public ShooterTuningClass(GamepadEx ct2, TelemetryCustom telemetry, LimeLight limelight) {
        this.ct2 = ct2;
        this.telemetry = telemetry;
        this.limelight = limelight;
        LoadDefaultTable();
        InitButtons();
    }

    private void InitButtons() {
        DpadUp    = new ButtonReader(ct2, GamepadKeys.Button.DPAD_UP);
        DpadDown  = new ButtonReader(ct2, GamepadKeys.Button.DPAD_DOWN);
        DpadLeft  = new ButtonReader(ct2, GamepadKeys.Button.DPAD_LEFT);
        DpadRight = new ButtonReader(ct2, GamepadKeys.Button.DPAD_RIGHT);
        BumpLeft  = new ButtonReader(ct2, GamepadKeys.Button.LEFT_BUMPER);
        BumpRight = new ButtonReader(ct2, GamepadKeys.Button.RIGHT_BUMPER);
        BButton   = new ButtonReader(ct2, GamepadKeys.Button.B);
        XButton   = new ButtonReader(ct2, GamepadKeys.Button.X);
        YButton   = new ButtonReader(ct2, GamepadKeys.Button.Y);
        SaveButton = new ButtonReader(ct2, GamepadKeys.Button.LEFT_STICK_BUTTON);
    }

    // ── Main Update (call every loop) ────────────────────────
    // Returns true if a test shoot was just triggered
    public boolean Update(double currentMotorVelocity) {
        ReadAllButtons();

        if (table.isEmpty()) return false;

        ShotParameters row = table.get(selectedRow);
        double step = stepSizes[stepIndex];

        // ── Navigate rows ──
        if (DpadUp.wasJustPressed()) {
            selectedRow = Math.max(0, selectedRow - 1);
        }
        if (DpadDown.wasJustPressed()) {
            selectedRow = Math.min(table.size() - 1, selectedRow + 1);
        }

        // ── Adjust RPM ──
        if (DpadRight.wasJustPressed()) {
            row.rpm += step;
        }
        if (DpadLeft.wasJustPressed()) {
            row.rpm -= step;
        }

        // ── Adjust Hood ──
        if (BumpRight.wasJustPressed()) {
            row.hoodAngle = Clamp(row.hoodAngle + (step > 1 ? 0.005 : step * 0.001 + 0.001), 0.0, 1.0);
        }
        if (BumpLeft.wasJustPressed()) {
            row.hoodAngle = Clamp(row.hoodAngle - (step > 1 ? 0.005 : step * 0.001 + 0.001), 0.0, 1.0);
        }

        // ── Cycle step ──
        if (BButton.wasJustPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // ── Add new row at current distance ──
        if (XButton.wasJustPressed()) {
            double dist = limelight.GetDistanceToTargetPython() * 100;
            if (dist > 0) {
                // Interpolate a starting point from existing table
                ShotParameters interpolated = GetShotParams(dist);
                table.add(new ShotParameters(dist, interpolated.rpm, interpolated.hoodAngle));
                SortTable();
                // Select the new row
                for (int i = 0; i < table.size(); i++) {
                    if (Math.abs(table.get(i).distanceCm - dist) < 1.0) {
                        selectedRow = i;
                        break;
                    }
                }
                telemetry.Log("✓ Row added at", String.format("%.1f cm", dist));
            } else {
                telemetry.Log("✗ No limelight distance", "Move closer to target");
            }
        }

        // ── Save current RPM + hood into selected row ──
        if (SaveButton.wasJustPressed()) {
            // Row is already being edited in-place, this just confirms + logs
            telemetry.Log("✓ Saved row", String.format("%.0fcm → %.0f RPM, hood %.3f",
                    row.distanceCm, row.rpm, row.hoodAngle));
            PrintFullTable();
        }

        // ── Test shoot ──
        testShootRequested = false;
        if (YButton.wasJustPressed()) {
            testShootRequested = true;
            testShootTimer.reset();
            testShootActive = true;
        }
        if (testShootActive && testShootTimer.seconds() > 1.0) {
            testShootActive = false;
        }

        // ── Telemetry ──
        LogTuning(row, currentMotorVelocity);

        return testShootRequested;
    }

    public void UpdateTurretTuningLive() {
        ReadAllButtons();

        if (BButton.wasJustPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        double step = stepSizes[stepIndex];
        
        if (DpadUp.wasJustPressed())    TurretProfiledPIDControl.Kp += step;
        if (DpadDown.wasJustPressed())  TurretProfiledPIDControl.Kp -= step;
        if (DpadRight.wasJustPressed()) TurretProfiledPIDControl.Ki += step;
        if (DpadLeft.wasJustPressed())  TurretProfiledPIDControl.Ki -= step;
        if (BumpRight.wasJustPressed()) TurretProfiledPIDControl.Kd += step;
        if (BumpLeft.wasJustPressed())  TurretProfiledPIDControl.Kd -= step;
    }

    public double getStepSize() {
        return stepSizes[stepIndex];
    }

    // ── Returns the currently selected row's values (for motor/hood control) ─
    public ShotParameters GetSelectedRow() {
        if (table.isEmpty()) return new ShotParameters(0, 1300, 0.51);
        return table.get(selectedRow);
    }

    // ── Telemetry ────────────────────────────────────────────
    private void LogTuning(ShotParameters row, double currentVel) {
        telemetry.Log("=== TUNING MODE ===", String.format("Row %d / %d", selectedRow + 1, table.size()));
        telemetry.Log("Step (B)", stepSizes[stepIndex]);
        telemetry.Log("Distance", String.format("%.1f cm", row.distanceCm));
        telemetry.Log("Target RPM  (DPAD L/R)", String.format("%.1f", row.rpm));
        telemetry.Log("Current RPM", String.format("%.1f", currentVel));
        telemetry.Log("Hood Angle  (BUMP L/R)", String.format("%.4f", row.hoodAngle));
        telemetry.Log("Limelight dist", String.format("%.1f cm", limelight.GetDistanceToTargetPython() * 100));
        telemetry.Log("X = add row | LStick = save | Y = test shoot", "");

        // Mini table preview (3 rows around selected)
        telemetry.Log("--- Table ---", "");
        int start = Math.max(0, selectedRow - 1);
        int end = Math.min(table.size() - 1, selectedRow + 1);
        for (int i = start; i <= end; i++) {
            ShotParameters r = table.get(i);
            String marker = (i == selectedRow) ? "►" : " ";
            telemetry.Log(marker + String.format(" %.0fcm", r.distanceCm),
                    String.format("%.0f RPM | %.3f hood", r.rpm, r.hoodAngle));
        }
    }

    private void PrintFullTable() {
        telemetry.Log("=== FULL TABLE ===", "copy these values");
        for (int i = 0; i < table.size(); i++) {
            ShotParameters r = table.get(i);
            telemetry.Log(String.format("Row %d", i),
                    String.format("{ %.0f, %.1f, %.4f }", r.distanceCm, r.rpm, r.hoodAngle));
        }
    }

    private void ReadAllButtons() {
        DpadUp.readValue();
        DpadDown.readValue();
        DpadLeft.readValue();
        DpadRight.readValue();
        BumpLeft.readValue();
        BumpRight.readValue();
        BButton.readValue();
        XButton.readValue();
        YButton.readValue();
        SaveButton.readValue();
    }

    private double Clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}