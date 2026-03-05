package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterVoltageHelper {

    // ================= CONFIG =================
    private static final double V_REF = 12.5; // Reference point at 12.5V where it works best
    private static final double COMPENSATION_GAIN = 1.1; // Lowered to prevent aggressive overshooting
    private static final double ALPHA = 0.15; // Slightly faster filtering
    private static final double MIN_SCALE = 0.85; // Limita de siguranta minimă
    private static final double MAX_SCALE = 1.15; // Limita de siguranta maximă

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
     * Acum rulează filtrarea în fiecare loop pentru fluiditate
     */
    public double GetCompensatedF(double baseF) {
        // Filtrăm voltajul în fiecare apel pentru tranzitii line
        double rawVoltage = GetBatteryVoltage();
        filteredVoltage += ALPHA * (rawVoltage - filteredVoltage);

        // Folosim o formulă de putere pentru a compensa pierderile non-liniare ale
        // motorului
        // (V_REF / V_actual) ^ GAIN
        double scale = Math.pow(V_REF / filteredVoltage, COMPENSATION_GAIN);
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
     * Force immediate F calculation (bypass filter)
     */
    public double ForceUpdate(double baseF) {
        double rawVoltage = GetBatteryVoltage();
        filteredVoltage = rawVoltage;

        double scale = Math.pow(V_REF / filteredVoltage, COMPENSATION_GAIN);
        scale = clamp(scale, MIN_SCALE, MAX_SCALE);

        lastCompensatedF = baseF * scale;
        lastUpdateTime = (long) timer.milliseconds();

        return lastCompensatedF;
    }

    // ================= INTERNAL =================

    private double GetBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        if (hardwareMap.voltageSensor == null)
            return V_REF;

        for (VoltageSensor v : hardwareMap.voltageSensor) {
            double val = v.getVoltage();
            if (val > 1.0)
                min = Math.min(min, val);
        }
        return min == Double.POSITIVE_INFINITY ? V_REF : min;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
