package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Valorile PID ale turelei gasite de TurretAutoTuner, salvate pe Control Hub
 * (fisierul FIRST/settings/turret_pid.txt). Raman si dupa restart.
 *
 * TurretMotionCompensatedPID si TurretBallisticShotPID le incarca la Initialize():
 * daca fisierul exista, valorile din el inlocuiesc Kp / Kd / kS scrise in cod.
 * Ca sa reveniti la valorile din cod: in tuner, butonul X sterge fisierul.
 *
 * Tot aici turela masoara cat dureaza bucla TeleOp-ului, ca tunerul sa testeze in aceleasi conditii.
 */
public final class TurretPidStore {
    private static final String FILE_NAME = "turret_pid.txt";

    /** Durata medie a buclei TeleOp (ms), masurata de turela. 0 = inca nemasurata in sesiunea asta. */
    public static double measuredLoopMs = 0;
    private static long lastNs = 0;

    /** true daca valorile din TeleOp au venit din fisier (pentru telemetrie). */
    public static boolean loadedFromFile = false;

    private TurretPidStore() { }

    /** Apelata de turela la fiecare bucla: masoara durata buclei. */
    public static void ReportLoop() {
        long now = System.nanoTime();
        if (lastNs != 0) {
            double ms = (now - lastNs) / 1e6;
            if (ms > 0 && ms < 200) { // pauzele mari (intre OpMode-uri) nu conteaza
                measuredLoopMs = measuredLoopMs == 0 ? ms : 0.95 * measuredLoopMs + 0.05 * ms;
            }
        }
        lastNs = now;
    }

    private static File File() {
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    /** {Kp, Kd, kS} din fisier, sau null daca nu exista / e stricat. */
    public static double[] Load() {
        try {
            File f = File();
            if (!f.exists()) return null;
            double kp = Double.NaN, kd = Double.NaN, ks = Double.NaN;
            for (String line : ReadWriteFile.readFile(f).split("\n")) {
                String[] kv = line.trim().split("=");
                if (kv.length != 2) continue;
                double v = Double.parseDouble(kv[1].trim());
                switch (kv[0].trim()) {
                    case "Kp": kp = v; break;
                    case "Kd": kd = v; break;
                    case "kS": ks = v; break;
                }
            }
            if (Double.isNaN(kp) || Double.isNaN(kd) || Double.isNaN(ks)) return null;
            return new double[] { kp, kd, ks };
        } catch (Exception e) {
            return null;
        }
    }

    public static boolean Save(double kp, double kd, double ks) {
        try {
            ReadWriteFile.writeFile(File(), String.format(java.util.Locale.US,
                    "Kp=%.5f\nKd=%.5f\nkS=%.4f\n", kp, kd, ks));
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    public static boolean Delete() {
        try {
            File f = File();
            return !f.exists() || f.delete();
        } catch (Exception e) {
            return false;
        }
    }
}
