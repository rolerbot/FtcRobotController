package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Corecteaza HEADING-ul Pinpoint-ului cu camera (Limelight, pipeline AprilTag nativ), doar cand e sigur.
 *
 * Cum calculeaza heading-ul:
 *  - din Pinpoint stim unde e turela pe teren (X/Y sunt stabile), deci si camera;
 *  - stim unde e tag-ul goal-ului pe teren (TAG_*), deci directia in care AR TREBUI sa fie tag-ul;
 *  - camera da tx = unghiul la care il vede DE FAPT;
 *  - heading = directia spre tag - unghiul turelei - zero-ul turelei - unghiul vazut de camera.
 *
 * Corecteaza doar cand TOATE conditiile sunt indeplinite:
 *  1. baza oprita (viteza X/Y si rotire mici), 2. turela nemiscata, 3. a trecut STILL_TIME_S de la oprire,
 *  4. tag-ul alliantei vazut, cadru proaspat, 5. distanta intre MIN_DIST_IN si MAX_DIST_IN,
 *  6. tag-ul nu e vazut prea din lateral, 7. SAMPLES citiri la rand, stabile (abatere < MAX_STD_DEG),
 *  8. diferenta (mediana) intre MIN_CORR_DEG si MAX_CORR_DEG, 9. minim COOLDOWN_S de la ultima corectie.
 * Corecteaza DOAR heading-ul; X/Y raman cele din Pinpoint.
 */
@Configurable
public class HeadingCameraCorrector {
    public static boolean ACTIVA = true;

    // Pozitia tag-urilor de goal pe teren (Pedro, inch), din biblioteca oficiala DECODE a SDK-ului
    // (tag 20: -58.37, -55.64; tag 24: -58.37, 55.64 in coordonate FTC) convertita in Pedro.
    public static double TAG_BLUE_X = 16.36, TAG_BLUE_Y = 130.37;   // tag 20
    public static double TAG_RED_X = 127.64, TAG_RED_Y = 130.37;    // tag 24

    // Camera fata de AXA turelei, cu turela la 0° (din setarile Limelight: 0.262 m in fata centrului
    // robotului + 0.1009 m de la centru la axa turelei; LL Right -0.002 m)
    public static double CAM_FWD_IN = 14.29;
    public static double CAM_LEFT_IN = 0.08;
    public static double CAM_PITCH_DEG = 15.0;      // camera inclinata in sus (LL Pitch)
    public static double CAM_YAW_OFFSET_DEG = 0.0;  // daca camera nu e montata perfect drept (calibrare)

    // conditii
    public static double BASE_MAX_SPEED_IN_S = 0.8;   // ~2 cm/s
    public static double BASE_MAX_TURN_DEG_S = 2.0;
    public static double TURRET_MAX_DEG_S = 1.0;
    public static double STILL_TIME_S = 0.3;
    public static double MIN_DIST_IN = 20.0;
    public static double MAX_DIST_IN = 100.0;         // ~2.5 m
    public static double MAX_VIEW_ANGLE_DEG = 45.0;   // cat de din lateral poate fi vazut tag-ul
    public static double MAX_STALENESS_MS = 100.0;
    public static int SAMPLES = 12;
    public static double MAX_STD_DEG = 0.4;
    public static double MIN_CORR_DEG = 0.75;
    public static double MAX_CORR_DEG = 10.0;
    public static double COOLDOWN_S = 1.0;

    private final RobotPinpoint rp;
    private final LimeLight limelight;
    private final TurretShot turret;

    private final ElapsedTime stillTimer = new ElapsedTime();
    private final ElapsedTime cooldown = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastTurretAngle = Double.NaN;
    private final List<Double> samples = new ArrayList<>();   // diferente (camera - Pinpoint), grade

    private String status = "-";
    private int corrections = 0;
    private double lastCorrection = 0, lastEstDiff = Double.NaN, lastStd = Double.NaN, lastDist = 0;

    public HeadingCameraCorrector(RobotPinpoint rp, LimeLight limelight, TurretShot turret) {
        this.rp = rp;
        this.limelight = limelight;
        this.turret = turret;
        cooldown.reset();
    }

    public void Run() {
        double dt = Math.max(loopTimer.seconds(), 1e-3);
        loopTimer.reset();
        double turretAngle = turret.getCurrentAngle();
        double turretRate = Double.isNaN(lastTurretAngle) ? 999 : Math.abs(turretAngle - lastTurretAngle) / dt;
        lastTurretAngle = turretAngle;

        if (!ACTIVA) { Clear("oprita (ACTIVA = false)"); stillTimer.reset(); return; }

        // 1-3: baza si turela oprite, de suficient timp
        boolean baseStill = Math.hypot(rp.GetVelX(), rp.GetVelY()) < BASE_MAX_SPEED_IN_S
                && Math.abs(Math.toDegrees(rp.GetHeadingVelocityRad())) < BASE_MAX_TURN_DEG_S;
        if (!baseStill) { stillTimer.reset(); Clear("baza se misca"); return; }
        if (turretRate > TURRET_MAX_DEG_S) { stillTimer.reset(); Clear("turela se misca"); return; }
        if (stillTimer.seconds() < STILL_TIME_S) { status = "astept sa se linisteasca"; return; }

        // 4: tag-ul alliantei, cadru proaspat
        if (!limelight.HasGoalTag()) { Clear("nu vad tag-ul goal-ului"); return; }
        if (limelight.GetStalenessMs() > MAX_STALENESS_MS) { Clear("cadru vechi de la camera"); return; }

        double tagX = rp.IsBlue() ? TAG_BLUE_X : TAG_RED_X;
        double tagY = rp.IsBlue() ? TAG_BLUE_Y : TAG_RED_Y;
        double zero = turret.getZeroHeadingDeg();

        // azimutul tag-ului fata de directia camerei (tx + corectie pentru camera inclinata)
        double tx = Math.toRadians(limelight.GetGoalTx());
        double ty = Math.toRadians(limelight.GetGoalTy());
        double p = Math.toRadians(CAM_PITCH_DEG);
        double camAz = Math.toDegrees(Math.atan2(-Math.tan(tx), Math.cos(p) - Math.tan(ty) * Math.sin(p)));

        // heading estimat (iteram: pozitia camerei depinde putin de heading)
        double h = rp.GetHeading();
        double camX = 0, camY = 0;
        for (int i = 0; i < 3; i++) {
            double camDir = Math.toRadians(h + zero + turretAngle);
            camX = rp.GetTurretX() + CAM_FWD_IN * Math.cos(camDir) - CAM_LEFT_IN * Math.sin(camDir);
            camY = rp.GetTurretY() + CAM_FWD_IN * Math.sin(camDir) + CAM_LEFT_IN * Math.cos(camDir);
            double bearing = Math.toDegrees(Math.atan2(tagY - camY, tagX - camX));
            h = Norm(bearing - camAz - CAM_YAW_OFFSET_DEG - zero - turretAngle);
        }

        // 5: distanta
        lastDist = Math.hypot(tagX - camX, tagY - camY);
        if (lastDist < MIN_DIST_IN) { Clear(String.format("prea aproape (%.0f\")", lastDist)); return; }
        if (lastDist > MAX_DIST_IN) { Clear(String.format("prea departe (%.0f\")", lastDist)); return; }

        // 6: unghiul din care e vazut tag-ul (normala tag-ului ~ spre centrul terenului)
        double normal = Math.atan2(72 - tagY, 72 - tagX);
        double toCam = Math.atan2(camY - tagY, camX - tagX);
        double view = Math.abs(Norm(Math.toDegrees(toCam - normal)));
        if (view > MAX_VIEW_ANGLE_DEG) { Clear(String.format("tag vazut prea din lateral (%.0f°)", view)); return; }

        // 7: citiri stabile (fereastra glisanta)
        double diff = Norm(h - rp.GetHeading());
        samples.add(diff);
        while (samples.size() > SAMPLES) samples.remove(0);
        if (samples.size() < SAMPLES) { status = String.format("colectez %d/%d", samples.size(), SAMPLES); return; }

        double med = Median(samples);
        lastStd = Std(samples);
        lastEstDiff = med;
        if (lastStd > MAX_STD_DEG) { status = String.format("citiri instabile (abatere %.2f°)", lastStd); return; }

        // 8-9
        if (Math.abs(med) < MIN_CORR_DEG) { status = String.format("OK: heading bun (diferenta %+.2f°)", med); return; }
        if (Math.abs(med) > MAX_CORR_DEG) { Clear(String.format("diferenta prea mare (%+.1f°) - ignor", med)); return; }
        if (cooldown.seconds() < COOLDOWN_S) { status = "pauza intre corectii"; return; }

        rp.SetPosition(rp.GetX(), rp.GetY(), Norm(rp.GetHeading() + med));
        corrections++;
        lastCorrection = med;
        cooldown.reset();
        stillTimer.reset();
        samples.clear();
        status = String.format("CORECTAT %+.2f°", med);
    }

    private void Clear(String why) {
        samples.clear();
        status = why;
    }

    private static double Median(List<Double> v) {
        double[] a = new double[v.size()];
        for (int i = 0; i < a.length; i++) a[i] = v.get(i);
        Arrays.sort(a);
        int n = a.length;
        return n % 2 == 1 ? a[n / 2] : 0.5 * (a[n / 2 - 1] + a[n / 2]);
    }

    private static double Std(List<Double> v) {
        double m = 0;
        for (double x : v) m += x;
        m /= v.size();
        double s = 0;
        for (double x : v) s += (x - m) * (x - m);
        return Math.sqrt(s / v.size());
    }

    private static double Norm(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    public void UpdateTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("=== CORECTIE HEADING CAMERA ===");
        telemetry.addData("  Stare", status);
        telemetry.addData("  Camera - Pinpoint", Double.isNaN(lastEstDiff) ? "-"
                : String.format("%+.2f° (abatere %.2f°, dist %.0f\")", lastEstDiff, lastStd, lastDist));
        telemetry.addData("  Corectii", "%d (ultima %+.2f°)", corrections, lastCorrection);
    }
}
