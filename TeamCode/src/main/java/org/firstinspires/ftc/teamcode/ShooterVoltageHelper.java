package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterVoltageHelper
{

    // ================= CONFIG =================
    private static final double V_REF = 12.0;     // voltajul la care ai făcut tuning
    private static final double ALPHA = 0.02;     // filtrare lenta (ignora spike-uri)
    private static final double MIN_SCALE = 0.90; // clamp sigur
    private static final double MAX_SCALE = 1.10;
    private static final long UPDATE_MS = 500;    // update rar PIDF

    // ================= STATE =================
    private final HardwareMap hardwareMap;
    private double filteredVoltage = V_REF;
    private double lastCompensatedF;
    private long lastUpdateTime = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public ShooterVoltageHelper(HardwareMap hwMap, double baseF) {
        this.hardwareMap = hwMap;
        this.lastCompensatedF = baseF;
        timer.reset();
    }

    // ================= PUBLIC =================

    /**
     * Returneaza F compensat la baterie (FILTRAT)
     * Se recomanda apel rar (ex: la 500ms)
     */
    public double GetCompensatedF(double baseF) {
        if (timer.milliseconds() - lastUpdateTime < UPDATE_MS) {
            return lastCompensatedF;
        }

        double rawVoltage = GetBatteryVoltage();
        filteredVoltage += ALPHA * (rawVoltage - filteredVoltage);

        double scale = V_REF / filteredVoltage;
        scale = clamp(scale, MIN_SCALE, MAX_SCALE);

        lastCompensatedF = baseF * scale;
        lastUpdateTime = (long) timer.milliseconds();

        return lastCompensatedF;
    }

    /**
     * Voltaj filtrat (doar pentru debug/telemetry)
     */
    public double GetFilteredVoltage() {
        return filteredVoltage;
    }

    /**
     * Get current battery voltage (raw, unfiltered)
     */
    public double GetRawVoltage() {
        return GetBatteryVoltage();
    }

    /**
     * Force immediate F calculation (bypass 500ms timer)
     * Use this during initialization to get instant compensation
     */
    public double ForceUpdate(double baseF) {
        double rawVoltage = GetBatteryVoltage();
        filteredVoltage = rawVoltage;  // Skip filtering for instant result

        double scale = V_REF / filteredVoltage;
        scale = clamp(scale, MIN_SCALE, MAX_SCALE);

        lastCompensatedF = baseF * scale;
        lastUpdateTime = (long) timer.milliseconds();

        return lastCompensatedF;
    }

    // ================= INTERNAL =================

    private double GetBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor v : hardwareMap.voltageSensor) {
            double val = v.getVoltage();
            if (val > 0) min = Math.min(min, val);
        }
        return min == Double.POSITIVE_INFINITY ? V_REF : min;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}

