package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

/**
 * Auto-tuning AUTOMAT pentru PID-ul turelei.
 *
 * Porneste SINGUR dupa START (PORNIRE_AUTOMATA). A = il porneste din nou (dureaza ~3-4 minute). B = opreste. X = sterge valorile salvate (revine la cele din cod).
 * Y = verifica doar valorile actuale (un singur test, fara sa schimbe nimic).
 *
 * Ce face singur:
 *  1. Masoara kS (puterea la care turela se urneste).
 *  2. Testeaza valorile actuale (ca referinta).
 *  3. Creste Kp pas cu pas pana turela incepe sa oscileze / sa treaca de tinta, apoi alege
 *     o valoare cu marja de siguranta (KP_MARJA) sub cea la care a aparut problema.
 *  4. Cauta Kd-ul cu cel mai bun rezultat pentru Kp-ul ales.
 *  5. Verifica de 2 ori valorile alese; daca apare oscilatie, scade Kp si reia.
 *  6. Le aplica si le SALVEAZA pe Control Hub -> TeleOp-ul le foloseste automat, si dupa restart.
 *
 * Fiecare test ruleaza cu ACEEASI viteza a buclei ca TeleOp-ul (masurata de turela in TeleOp)
 * si cu acelasi controler (profil, deadzone, toleranta, kS) - altfel valorile bune aici
 * oscileaza in TeleOp. Daca TeleOp-ul n-a rulat inca de la pornirea aplicatiei, se foloseste
 * LOOP_MS_IMPLICIT.
 *
 * INAINTE DE INIT: turela la opritorul din DREAPTA (acolo encoderul devine 0), robotul pe loc.
 */
@Configurable
@TeleOp(name = "Turela - Auto Tuning PID", group = "Tuning")
public class TurretAutoTuner extends LinearOpMode {

    private static final String VERSIUNE = "v2 (automat)";

    // ---- reglabile din Panels ----
    public static boolean PORNIRE_AUTOMATA = true;   // true = auto-tuning-ul porneste singur dupa START
    public static double LOOP_MS_IMPLICIT = 30.0;   // bucla TeleOp, daca nu e masurata
    public static double LOOP_MARJA = 1.25;         // testam cu bucla putin mai lenta decat in TeleOp
    public static double KP_START = 0.02;
    public static double KP_PAS = 1.3;              // Kp creste cu 30% la fiecare incercare
    public static double KP_MAX = 0.6;
    public static double KP_MARJA = 0.7;            // Kp ales = 70% din ultimul Kp bun
    public static double MAX_DEPASIRE_DEG = 1.5;    // peste asta = prea agresiv
    public static double BANDA_STABIL_DEG = 1.0;    // "a ajuns" = eroare sub asta (cel putin toleranta din TeleOp)
    public static int MAX_SCHIMBARI_SENS = 3;       // schimbari de sens ale puterii cat sta pe tinta = oscilatie
    public static double TINUT_S = 1.3;             // cat timp se urmareste fiecare mutare
    public static double URMARIRE_VITEZA_DEG_S = 60;// tinta in miscare (ca robotul care se roteste)
    public static double SAFE_LIMIT_DEG = 65.0;
    public static double KS_RAMP_PER_S = 0.05;
    public static double KS_MOVE_DEG_S = 4.0;

    private static final double[] KD_CANDIDATI = { 0.0, 0.002, 0.004, 0.006, 0.009, 0.013 };
    private static final double[] MUTARI = { 12, -12, 0, 30, 0 };

    private DcMotorEx motor;
    private final ElapsedTime pace = new ElapsedTime();
    private double loopMs = 30;
    private boolean aborted = false;

    private String stage = "Gata. A = porneste auto-tuning-ul.";
    private final List<String> log = new ArrayList<>();
    private String summary = "";
    private boolean prevA, prevB, prevX, prevY;

    /** Rezultatul unui test complet. */
    private static class Result {
        boolean ok = true;
        boolean unstable = false;   // true = prea agresiv (depaseste / oscileaza); false = prea slab
        String why = "";
        double overshoot = 0, settleSum = 0, trackRms = 0;
        int moves = 0, flips = 0, chatter = 0;
        double settleMean() { return moves == 0 ? 99 : settleSum / moves; }
        double score() { return settleMean() + 0.2 * overshoot + 0.1 * trackRms; }
        String text() {
            return ok ? String.format("OK  stab %.2fs  dep %.1f°  urm %.1f°", settleMean(), overshoot, trackRms)
                      : "RESPINS: " + why;
        }
    }

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "MotorTurela");
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        loopMs = (TurretPidStore.measuredLoopMs > 3 ? TurretPidStore.measuredLoopMs : LOOP_MS_IMPLICIT) * LOOP_MARJA;

        telemetry.addLine("AUTO TUNING TURELA " + VERSIUNE);
        telemetry.addLine("Turela trebuie sa fie la opritorul din DREAPTA (encoder = 0).");
        telemetry.addLine(PORNIRE_AUTOMATA ? "Dupa START porneste SINGUR (~3-4 min). B = opreste."
                                           : "Dupa START apasa A. B = opreste.");
        telemetry.addLine("Robotul pe loc, nimeni langa turela.");
        telemetry.addData("Bucla TeleOp", TurretPidStore.measuredLoopMs > 3
                ? String.format("%.0f ms (masurata) -> testez la %.0f ms", TurretPidStore.measuredLoopMs, loopMs)
                : String.format("nemasurata -> testez la %.0f ms. (Mai precis: ruleaza intai TeleOp-ul 20 s.)", loopMs));
        telemetry.update();
        waitForStart();

        GoTo(0, 2.0);
        if (PORNIRE_AUTOMATA && opModeIsActive()) SafeAutoTune();

        while (opModeIsActive()) {
            // merg ambele controllere (gamepad 1 sau 2)
            boolean a = gamepad1.a || gamepad2.a, b = gamepad1.b || gamepad2.b;
            boolean x = gamepad1.x || gamepad2.x, y = gamepad1.y || gamepad2.y;
            if (a && !prevA) SafeAutoTune();
            if (y && !prevY) {
                stage = "Verific valorile actuale...";
                Result r = Evaluate(TurretMotionCompensatedPID.Kp, TurretMotionCompensatedPID.Kd, TurretMotionCompensatedPID.kS);
                AddLog(String.format("Actual Kp %.4f Kd %.4f kS %.3f: %s", TurretMotionCompensatedPID.Kp,
                        TurretMotionCompensatedPID.Kd, TurretMotionCompensatedPID.kS, r.text()));
                stage = "Gata.";
                GoTo(0, 1.5);
            }
            if (x && !prevX) {
                boolean ok = TurretPidStore.Delete();
                stage = ok ? "Fisierul cu valori a fost sters. La urmatorul TeleOp se folosesc valorile din cod."
                           : "Nu am putut sterge fisierul.";
            }
            prevA = a; prevB = b; prevX = x; prevY = y;
            aborted = false;
            Hold();
            Show();
        }
        motor.setPower(0);
    }

    // ======================================================================
    //                              AUTO-TUNING
    // ======================================================================

    /** Porneste auto-tuning-ul; orice eroare e afisata pe ecran in loc sa opreasca OpMode-ul. */
    private void SafeAutoTune() {
        stage = "A pornit auto-tuning-ul...";
        Show();
        try {
            RunAutoTune();
        } catch (Exception e) {
            Off();
            stage = "EROARE: " + e;
        }
        holdController = null;
    }

    private void RunAutoTune() {
        aborted = false;
        log.clear();
        summary = "";

        // 1. kS
        stage = "1/5 Masor kS (pornire)...";
        double ksPlus = RampUntilMoves(+1);
        if (Stop()) return;
        double ksMinus = RampUntilMoves(-1);
        if (Stop()) return;
        if (Double.isNaN(ksPlus) || Double.isNaN(ksMinus)) {
            stage = "OPRIT: turela nu se misca nici la putere 0.5. Verificati mecanic.";
            return;
        }
        double ks = 0.8 * Math.min(ksPlus, ksMinus);
        AddLog(String.format("kS: + %.3f / - %.3f -> folosesc %.3f", ksPlus, ksMinus, ks));

        // 2. referinta: valorile actuale
        stage = "2/5 Testez valorile actuale (referinta)...";
        double oldKp = TurretMotionCompensatedPID.Kp, oldKd = TurretMotionCompensatedPID.Kd, oldKs = TurretMotionCompensatedPID.kS;
        Result before = Evaluate(oldKp, oldKd, oldKs);
        if (Stop()) return;
        AddLog(String.format("Inainte Kp %.4f Kd %.4f kS %.3f: %s", oldKp, oldKd, oldKs, before.text()));

        // 3. Kp: crestem pana apare problema
        double lastGoodKp = Double.NaN;
        double kp = KP_START;
        while (kp <= KP_MAX) {
            stage = String.format("3/5 Caut Kp: incerc %.4f ...", kp);
            Result r = Evaluate(kp, 0.0, ks);
            if (Stop()) return;
            AddLog(String.format("Kp %.4f: %s", kp, r.text()));
            if (r.ok) lastGoodKp = kp;
            else if (r.unstable) break;   // prea agresiv -> ne oprim; prea slab -> crestem in continuare
            kp *= KP_PAS;
        }
        if (Double.isNaN(lastGoodKp)) {
            stage = "OPRIT: nici cel mai mic Kp nu a mers. Scade KP_START sau verifica turela.";
            return;
        }
        double chosenKp = lastGoodKp * KP_MARJA;
        AddLog(String.format("Ultimul Kp bun %.4f -> aleg %.4f (marja %.0f%%)", lastGoodKp, chosenKp, KP_MARJA * 100));

        // 4. Kd
        double bestKd = 0;
        Result best = null;
        for (double kd : KD_CANDIDATI) {
            stage = String.format("4/5 Caut Kd: incerc %.4f (Kp %.4f)...", kd, chosenKp);
            Result r = Evaluate(chosenKp, kd, ks);
            if (Stop()) return;
            AddLog(String.format("Kd %.4f: %s", kd, r.text()));
            if (r.ok && (best == null || r.score() < best.score())) {
                best = r;
                bestKd = kd;
            }
        }
        if (best == null) bestKd = 0;

        // 5. verificare de 2 ori; daca oscileaza, scadem Kp
        for (int attempt = 0; attempt < 4; attempt++) {
            stage = String.format("5/5 Verific Kp %.4f Kd %.4f kS %.3f (%d)...", chosenKp, bestKd, ks, attempt + 1);
            Result r1 = Evaluate(chosenKp, bestKd, ks);
            if (Stop()) return;
            Result r2 = r1.ok ? Evaluate(chosenKp, bestKd, ks) : r1;
            if (Stop()) return;
            AddLog(String.format("Verificare: %s | %s", r1.text(), r2.text()));
            if (r1.ok && r2.ok) {
                Apply(chosenKp, bestKd, ks);
                summary = String.format("NOU:     Kp %.4f  Kd %.4f  kS %.3f -> %s\nINAINTE: Kp %.4f  Kd %.4f  kS %.3f -> %s",
                        chosenKp, bestKd, ks, r1.text(), oldKp, oldKd, oldKs, before.text());
                stage = "GATA. Valorile sunt aplicate si salvate: porniti TeleOp-ul.";
                GoTo(0, 1.5);
                return;
            }
            chosenKp *= 0.8;
        }
        stage = "OPRIT: nu am gasit valori stabile. Valorile vechi raman neschimbate.";
        GoTo(0, 1.5);
    }

    private void Apply(double kp, double kd, double ks) {
        TurretMotionCompensatedPID.Kp = kp;
        TurretMotionCompensatedPID.Kd = kd;
        TurretMotionCompensatedPID.kS = ks;
        TurretBallisticShotPID.Kp = kp;
        TurretBallisticShotPID.Kd = kd;
        TurretBallisticShotPID.kS = ks;
        boolean saved = TurretPidStore.Save(kp, kd, ks);
        AddLog(saved ? "Salvat pe Control Hub (se pastreaza si dupa restart)." : "ATENTIE: nu am putut salva fisierul!");
    }

    // ======================================================================
    //                    UN TEST COMPLET (mutari + urmarire)
    // ======================================================================

    /** Ruleaza mutarile din MUTARI si urmarirea unei tinte in miscare, exact ca in TeleOp. */
    private Result Evaluate(double kp, double kd, double ks) {
        Result res = new Result();
        GoTo(0, 1.5);
        if (Stop()) { res.ok = false; res.why = "oprit"; return res; }

        ProfiledPIDController c = NewController(kp, kd);
        c.reset(Angle());
        double band = Math.max(BANDA_STABIL_DEG, TurretMotionCompensatedPID.ANGLE_TOLERANCE_DEG + 0.1);

        // --- mutari in trepte ---
        for (double target : MUTARI) {
            double start = Angle();
            double dir = Math.signum(target - start);
            double over = 0, settleAt = -1;
            int flips = 0;
            double lastSign = 0;
            ElapsedTime t = new ElapsedTime();
            while (opModeIsActive() && t.seconds() < TINUT_S) {
                if (CheckAbort()) { res.ok = false; res.why = "oprit"; return res; }
                double angle = Angle();
                if (Math.abs(angle) > SAFE_LIMIT_DEG) { Off(); res.ok = false; res.why = "a iesit din zona sigura"; return res; }
                double power = Control(c, angle, target, ks);
                motor.setPower(power);

                if (dir != 0) over = Math.max(over, (angle - target) * dir);
                if (over > 3 * MAX_DEPASIRE_DEG) { Off(); res.ok = false; res.unstable = true; res.why = String.format("depasire %.1f°", over); return res; }
                double err = target - angle;
                if (Math.abs(err) < band) { if (settleAt < 0) settleAt = t.seconds(); } else settleAt = -1;
                // schimbari de sens in a doua jumatate (cand ar trebui sa stea pe tinta)
                if (t.seconds() > TINUT_S / 2 && Math.abs(power) > 0.03) {
                    double s = Math.signum(power);
                    if (lastSign != 0 && s != lastSign) flips++;
                    lastSign = s;
                }
                Pace();
                ShowLive(String.format("Kp %.4f Kd %.4f kS %.3f | tinta %.0f° unghi %.1f°", kp, kd, ks, target, angle));
            }
            res.overshoot = Math.max(res.overshoot, over);
            res.flips = Math.max(res.flips, flips);
            res.moves++;
            res.settleSum += settleAt < 0 ? TINUT_S : settleAt;
            if (over > MAX_DEPASIRE_DEG) { res.ok = false; res.unstable = true; res.why = String.format("depasire %.1f°", over); }
            else if (flips > MAX_SCHIMBARI_SENS) { res.ok = false; res.unstable = true; res.why = "oscileaza pe tinta (" + flips + " schimbari)"; }
            else if (settleAt < 0) { res.ok = false; res.why = String.format("nu ajunge la %.0f°", target); }
            if (!res.ok) { Off(); return res; }
        }

        // --- urmarire: tinta care se misca inainte-inapoi (ca la rotirea robotului) ---
        double amp = 20, period = 4 * amp / URMARIRE_VITEZA_DEG_S;
        double sumSq = 0; int n = 0;
        double lastSign = 0, lastFlipT = -1;
        ElapsedTime t = new ElapsedTime();
        double persistent = Angle();
        while (opModeIsActive() && t.seconds() < 4.0) {
            if (CheckAbort()) { res.ok = false; res.why = "oprit"; return res; }
            double angle = Angle();
            if (Math.abs(angle) > SAFE_LIMIT_DEG) { Off(); res.ok = false; res.why = "a iesit din zona sigura"; return res; }
            double ph = (t.seconds() % period) / period;
            double needed = ph < 0.5 ? -amp + 4 * amp * ph : 3 * amp - 4 * amp * ph; // triunghi -amp..amp
            // aceeasi deadzone ca in TeleOp (cea mai mica, adica cazul cel mai sensibil)
            if (Math.abs(angle - needed) > TurretMotionCompensatedPID.DEADZONE_FAR_DEG) persistent = needed;
            double power = Control(c, angle, persistent, ks);
            motor.setPower(power);
            if (t.seconds() > 0.6) { sumSq += (needed - angle) * (needed - angle); n++; }
            if (Math.abs(power) > 0.03) {
                double s = Math.signum(power);
                if (lastSign != 0 && s != lastSign) {
                    if (lastFlipT >= 0 && t.seconds() - lastFlipT < 0.15) res.chatter++;
                    lastFlipT = t.seconds();
                }
                lastSign = s;
            }
            Pace();
            ShowLive(String.format("Kp %.4f Kd %.4f | urmarire: tinta %.1f° unghi %.1f°", kp, kd, needed, angle));
        }
        Off();
        res.trackRms = n > 0 ? Math.sqrt(sumSq / n) : 99;
        if (res.chatter > 6) { res.ok = false; res.unstable = true; res.why = "tremura la urmarire (" + res.chatter + ")"; }
        return res;
    }

    // ======================================================================
    //                              UTILITARE
    // ======================================================================

    private ProfiledPIDController NewController(double kp, double kd) {
        return new ProfiledPIDController(kp, 0, kd, new TrapezoidProfile.Constraints(
                TurretMotionCompensatedPID.maxVelocity, TurretMotionCompensatedPID.maxAcceleration));
    }

    /** Acelasi calcul ca TurretMotionCompensatedPID.Update(): PID cu profil + kS peste toleranta + limita 0.6. */
    private double Control(ProfiledPIDController c, double angle, double target, double ks) {
        double power = c.calculate(angle, target);
        double err = target - angle;
        if (Math.abs(err) > TurretMotionCompensatedPID.ANGLE_TOLERANCE_DEG) power += Math.signum(err) * ks;
        return Range.clip(power, -0.6, 0.6);
    }

    /** Asteapta pana trece o bucla de durata TeleOp-ului. */
    private void Pace() {
        while (opModeIsActive() && pace.milliseconds() < loopMs) { /* asteptam */ }
        pace.reset();
    }

    /** Duce turela la unghi cu valorile vechi, sigure (Kp 0.032 + kS mic). */
    private void GoTo(double target, double timeS) {
        ProfiledPIDController c = NewController(0.032, 0.005);
        c.reset(Angle());
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeS) {
            if (CheckAbort()) break;
            motor.setPower(Control(c, Angle(), target, 0.05));
            Pace();
        }
        Off();
        holdController = null;
    }

    private double RampUntilMoves(int dir) {
        GoTo(-dir * 20.0, 2.0);
        ElapsedTime t = new ElapsedTime();
        double power = 0;
        while (opModeIsActive() && power < 0.5) {
            if (CheckAbort()) { Off(); return Double.NaN; }
            power = KS_RAMP_PER_S * t.seconds();
            motor.setPower(dir * power);
            double vel = motor.getVelocity() / TurretMotionCompensatedPID.TICKS_PER_DEGREE;
            if (Math.abs(vel) > KS_MOVE_DEG_S || Math.abs(Angle()) > SAFE_LIMIT_DEG) {
                Off();
                GoTo(0, 2.0);
                return power;
            }
            ShowLive(String.format("kS: sens %s, putere %.3f", dir > 0 ? "+" : "-", power));
        }
        Off();
        GoTo(0, 2.0);
        return Double.NaN;
    }

    private ProfiledPIDController holdController;
    private void Hold() {
        if (holdController == null) {
            holdController = NewController(TurretMotionCompensatedPID.Kp, TurretMotionCompensatedPID.Kd);
            holdController.reset(Angle());
        }
        motor.setPower(Control(holdController, Angle(), 0, TurretMotionCompensatedPID.kS));
    }

    private boolean CheckAbort() {
        if (gamepad1.b || gamepad2.b) aborted = true;
        return aborted || !opModeIsActive();
    }

    private boolean Stop() {
        if (aborted || !opModeIsActive()) {
            Off();
            stage = "OPRIT de la B. Valorile vechi raman neschimbate.";
            return true;
        }
        return false;
    }

    private void Off() { motor.setPower(0); }

    private double Angle() {
        return (motor.getCurrentPosition() - TurretMotionCompensatedPID.TICKS_AT_CENTER)
                / TurretMotionCompensatedPID.TICKS_PER_DEGREE;
    }

    private void AddLog(String s) {
        log.add(s);
        while (log.size() > 12) log.remove(0);
    }

    private void ShowLive(String live) {
        telemetry.addLine("=== AUTO TUNING TURELA " + VERSIUNE + " ===  (B = opreste)");
        telemetry.addData("Etapa", stage);
        telemetry.addData("Acum", live);
        for (String s : log) telemetry.addLine("  " + s);
        telemetry.update();
    }

    private void Show() {
        telemetry.addLine("=== AUTO TUNING TURELA " + VERSIUNE + " ===");
        telemetry.addData("Stare", stage);
        telemetry.addLine("A = auto-tuning | Y = verifica valorile actuale | X = sterge valorile salvate | B = opreste");
        telemetry.addData("Unghi", "%.1f°", Angle());
        telemetry.addData("Bucla testata", "%.0f ms (TeleOp %s)", loopMs,
                TurretPidStore.measuredLoopMs > 3 ? String.format("%.0f ms", TurretPidStore.measuredLoopMs) : "nemasurat");
        telemetry.addData("Actual", "Kp %.4f  Kd %.4f  kS %.3f", TurretMotionCompensatedPID.Kp,
                TurretMotionCompensatedPID.Kd, TurretMotionCompensatedPID.kS);
        if (!summary.isEmpty()) telemetry.addLine(summary);
        telemetry.addLine("--- jurnal ---");
        for (String s : log) telemetry.addLine("  " + s);
        telemetry.update();
    }
}
